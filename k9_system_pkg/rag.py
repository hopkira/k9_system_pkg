#!/usr/bin/env python3

import json
import os
from pathlib import Path
from typing import Any, Dict, List, Sequence

os.environ.setdefault("HF_HUB_OFFLINE", "1")
os.environ.setdefault("TRANSFORMERS_OFFLINE", "1")

import chromadb
import ollama
import rclpy
import torch
from rclpy.node import Node
from transformers import AutoModelForCausalLM, AutoTokenizer

from k9_interfaces_pkg.srv import RetrieveKnowledge


class K9RagNode(Node):
    """
    K9 long-term-memory retrieval service.

    Pipeline:
        query -> Qwen3 embedding -> Chroma top-N ->
        Qwen3 reranker -> single best passage.
    """

    def __init__(self) -> None:
        super().__init__("k9_rag")

        self.declare_parameter("ollama_host", "http://127.0.0.1:11434")
        self.declare_parameter("embed_model", "qwen3-embedding:4b")
        self.declare_parameter("database_path", "~/k9_data/chroma_db")
        self.declare_parameter("collection_name", "k9_ltm_qwen3_4b")
        self.declare_parameter("candidate_count", 5)
        self.declare_parameter("min_embedding_score", 0.30)
        self.declare_parameter(
            "query_instruction",
            "Retrieve relevant passages from K9's long-term memory that help answer the user's question.",
        )

        self.declare_parameter(
            "reranker_path",
            "~/k9_models/Qwen3-Reranker-0.6B",
        )
        self.declare_parameter("reranker_device", "auto")
        self.declare_parameter("reranker_max_length", 2048)
        self.declare_parameter("fallback_to_embedding", True)
        self.declare_parameter(
            "reranker_instruction",
            (
                "Given a question about K9's history or knowledge, determine "
                "whether the document contains information that directly helps "
                "answer the question."
            ),
        )

        self.ollama_host = str(self.get_parameter("ollama_host").value)
        self.embed_model = str(self.get_parameter("embed_model").value)
        self.database_path = os.path.expanduser(
            str(self.get_parameter("database_path").value)
        )
        self.collection_name = str(self.get_parameter("collection_name").value)
        self.candidate_count = max(
            1,
            int(self.get_parameter("candidate_count").value),
        )
        self.min_embedding_score = float(
            self.get_parameter("min_embedding_score").value
        )
        self.query_instruction = str(
            self.get_parameter("query_instruction").value
        )

        self.reranker_path = Path(
            os.path.expanduser(
                str(self.get_parameter("reranker_path").value)
            )
        )
        self.reranker_device_parameter = str(
            self.get_parameter("reranker_device").value
        )
        self.reranker_max_length = max(
            256,
            int(self.get_parameter("reranker_max_length").value),
        )
        self.fallback_to_embedding = bool(
            self.get_parameter("fallback_to_embedding").value
        )
        self.reranker_instruction = str(
            self.get_parameter("reranker_instruction").value
        )

        os.makedirs(self.database_path, exist_ok=True)

        self.ollama_client = ollama.Client(host=self.ollama_host)
        self.chroma_client = chromadb.PersistentClient(
            path=self.database_path
        )
        self.collection = self.chroma_client.get_or_create_collection(
            name=self.collection_name,
            metadata={"hnsw:space": "cosine"},
        )

        self.reranker_device = self._select_reranker_device()

        self.get_logger().info(
            "Loading local Qwen reranker from "
            f"{self.reranker_path} on {self.reranker_device}"
        )

        self._load_reranker()

        self.service = self.create_service(
            RetrieveKnowledge,
            "/k9/rag/retrieve",
            self.retrieve_callback,
        )

        self.get_logger().info(
            "K9 RAG ready: "
            f"embedding={self.embed_model}, "
            "reranker=Qwen3-Reranker-0.6B, "
            f"candidates={self.candidate_count}, "
            "returns=1, "
            f"collection={self.collection_name}, "
            f"documents={self.collection.count()}"
        )

    def _select_reranker_device(self) -> str:
        requested = self.reranker_device_parameter.strip().lower()

        if requested == "auto":
            return "cuda" if torch.cuda.is_available() else "cpu"

        if requested == "cuda" and not torch.cuda.is_available():
            raise RuntimeError(
                "reranker_device is 'cuda' but torch.cuda.is_available() is false"
            )

        return requested

    def _load_reranker(self) -> None:
        if not self.reranker_path.is_dir():
            raise RuntimeError(
                "Local reranker model not found at "
                f"{self.reranker_path}. Run download_reranker.py once."
            )

        self.reranker_tokenizer = AutoTokenizer.from_pretrained(
            str(self.reranker_path),
            padding_side="left",
            local_files_only=True,
        )

        self.reranker_model = AutoModelForCausalLM.from_pretrained(
            str(self.reranker_path),
            torch_dtype="auto",
            local_files_only=True,
        ).to(self.reranker_device)

        self.reranker_model.eval()

        self.false_token_id = (
            self.reranker_tokenizer.convert_tokens_to_ids("no")
        )
        self.true_token_id = (
            self.reranker_tokenizer.convert_tokens_to_ids("yes")
        )

        prefix = (
            "<|im_start|>system\n"
            "Judge whether the Document meets the requirements based on "
            "the Query and the Instruct provided. Note that the answer "
            "can only be \"yes\" or \"no\"."
            "<|im_end|>\n"
            "<|im_start|>user\n"
        )
        suffix = (
            "<|im_end|>\n"
            "<|im_start|>assistant\n"
            "<think>\n\n</think>\n\n"
        )

        self.reranker_prefix_tokens = self.reranker_tokenizer.encode(
            prefix,
            add_special_tokens=False,
        )
        self.reranker_suffix_tokens = self.reranker_tokenizer.encode(
            suffix,
            add_special_tokens=False,
        )

        self.get_logger().info("Qwen reranker loaded successfully")

    def _format_embedding_query(self, query: str) -> str:
        return (
            f"Instruct: {self.query_instruction}\n"
            f"Query:{query}"
        )

    def _embed_query(self, query: str) -> List[float]:
        response = self.ollama_client.embed(
            model=self.embed_model,
            input=self._format_embedding_query(query),
            keep_alive="5m",
        )

        embeddings = getattr(response, "embeddings", None)
        if embeddings is None:
            embeddings = response["embeddings"]

        if not embeddings:
            raise RuntimeError("Ollama returned no embedding")

        return list(embeddings[0])

    @staticmethod
    def _similarity_from_cosine_distance(distance: float) -> float:
        return 1.0 - float(distance)

    def _retrieve_candidates(
        self,
        query: str,
        count: int,
    ) -> List[Dict[str, Any]]:
        if self.collection.count() == 0:
            return []

        query_embedding = self._embed_query(query)

        results = self.collection.query(
            query_embeddings=[query_embedding],
            n_results=min(count, self.collection.count()),
            include=["documents", "metadatas", "distances"],
        )

        ids = (results.get("ids") or [[]])[0]
        documents = (results.get("documents") or [[]])[0]
        metadatas = (results.get("metadatas") or [[]])[0]
        distances = (results.get("distances") or [[]])[0]

        candidates: List[Dict[str, Any]] = []

        for rank, (doc_id, document, metadata, distance) in enumerate(
            zip(ids, documents, metadatas, distances),
            start=1,
        ):
            embedding_score = self._similarity_from_cosine_distance(
                distance
            )

            min_embedding_score = float(
                self.get_parameter(
                    "min_embedding_score"
                ).value
            )

            if embedding_score < min_embedding_score:
                continue

            candidates.append(
                {
                    "id": str(doc_id),
                    "document": document or "",
                    "metadata": metadata or {},
                    "embedding_score": embedding_score,
                    "embedding_rank": rank,
                }
            )

        return candidates

    def _format_reranker_pair(
        self,
        query: str,
        document: str,
    ) -> str:
        return (
            f"<Instruct>: {self.reranker_instruction}\n"
            f"<Query>: {query}\n"
            f"<Document>: {document}"
        )

    def _rerank(
        self,
        query: str,
        candidates: Sequence[Dict[str, Any]],
    ) -> List[float]:

        if not candidates:
            return []

        scores: List[float] = []

        payload_max_length = (
            self.reranker_max_length
            - len(self.reranker_prefix_tokens)
            - len(self.reranker_suffix_tokens)
        )

        if payload_max_length <= 0:
            raise RuntimeError(
                "reranker_max_length is too small"
            )

        for candidate in candidates:

            pair = self._format_reranker_pair(
                query,
                candidate["document"],
            )

            tokenized = self.reranker_tokenizer(
                pair,
                padding=False,
                truncation=True,
                return_attention_mask=False,
                max_length=payload_max_length,
            )

            input_ids = (
                self.reranker_prefix_tokens
                + tokenized["input_ids"]
                + self.reranker_suffix_tokens
            )

            inputs = self.reranker_tokenizer.pad(
                {
                    "input_ids": [
                        input_ids
                    ]
                },
                padding=True,
                return_tensors="pt",
            )

            inputs = {
                key: value.to(
                    self.reranker_device
                )
                for key, value in inputs.items()
            }

            with torch.inference_mode():

                final_logits = (
                    self.reranker_model(
                        **inputs
                    ).logits[:, -1, :]
                )

                true_vector = final_logits[
                    :,
                    self.true_token_id,
                ]

                false_vector = final_logits[
                    :,
                    self.false_token_id,
                ]

                yes_no_logits = torch.stack(
                    [
                        false_vector,
                        true_vector,
                    ],
                    dim=1,
                )

                probability = torch.softmax(
                    yes_no_logits,
                    dim=1,
                )[0, 1]

                scores.append(
                    float(
                        probability.detach().cpu()
                    )
                )

            # Release temporary tensors before evaluating
            # the next candidate.
            del inputs
            del final_logits
            del true_vector
            del false_vector
            del yes_no_logits
            del probability

            if (
                self.reranker_device == "cuda"
                and torch.cuda.is_available()
            ):
                torch.cuda.empty_cache()

        return scores
    

    def retrieve_callback(
        self,
        request: RetrieveKnowledge.Request,
        response: RetrieveKnowledge.Response,
    ) -> RetrieveKnowledge.Response:
        query = request.query.strip()

        if not query:
            response.success = False
            response.error = "Query is empty"
            return response

        # Preserve the existing service interface. max_results now controls
        # the candidate pool; this node intentionally returns only one result.
        candidate_count = (
            int(request.max_results)
            if request.max_results > 0
            else self.candidate_count
        )
        candidate_count = max(1, candidate_count)

        try:
            candidates = self._retrieve_candidates(
                query,
                candidate_count,
            )

            if not candidates:
                response.success = True
                response.error = ""
                self.get_logger().info(
                    "RAG: no embedding candidates above threshold: "
                    f"{query[:80]}"
                )
                return response

            try:
                reranker_scores = self._rerank(
                    query,
                    candidates,
                )

                for candidate, score in zip(
                    candidates,
                    reranker_scores,
                ):
                    candidate["reranker_score"] = score

                best = max(
                    candidates,
                    key=lambda item: item["reranker_score"],
                )

                final_score = float(
                    best["reranker_score"]
                )

            except Exception as exc:
                if not self.fallback_to_embedding:
                    raise

                self.get_logger().warning(
                    "Reranker failed; using best embedding candidate: "
                    f"{exc}"
                )

                best = max(
                    candidates,
                    key=lambda item: item["embedding_score"],
                )
                best["reranker_score"] = None
                final_score = float(best["embedding_score"])

            metadata = dict(best["metadata"])
            metadata["embedding_score"] = round(
                float(best["embedding_score"]),
                6,
            )
            metadata["embedding_rank"] = int(
                best["embedding_rank"]
            )

            if best.get("reranker_score") is not None:
                metadata["reranker_score"] = round(
                    float(best["reranker_score"]),
                    6,
                )

            metadata["retrieval_pipeline"] = (
                "qwen3-embedding:4b -> chroma top-N -> "
                "Qwen3-Reranker-0.6B -> top-1"
            )

            response.ids.append(best["id"])
            response.documents.append(best["document"])
            response.sources.append(
                str(metadata.get("source", ""))
            )
            response.scores.append(final_score)
            response.metadata_json.append(
                json.dumps(
                    metadata,
                    separators=(",", ":"),
                    sort_keys=True,
                )
            )

            response.success = True
            response.error = ""

            rerank_text = (
                f"{best['reranker_score']:.3f}"
                if best.get("reranker_score") is not None
                else "fallback"
            )

            self.get_logger().info(
                "RAG: "
                f"{len(candidates)} candidate(s), "
                f"winner embedding_rank={best['embedding_rank']}, "
                f"embedding={best['embedding_score']:.3f}, "
                f"reranker={rerank_text}, "
                f"source={metadata.get('source', '')}: "
                f"{query[:80]}"
            )

        except Exception as exc:
            self.get_logger().error(
                f"RAG retrieval failed: {exc}"
            )
            response.success = False
            response.error = str(exc)

        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = K9RagNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
