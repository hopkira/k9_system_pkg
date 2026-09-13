\
#!/usr/bin/env python3

import json
import os
from typing import Any, List

import chromadb
import ollama
import rclpy
from rclpy.node import Node

from k9_interfaces_pkg.srv import RetrieveKnowledge


class K9RagNode(Node):
    """ROS 2 service providing semantic retrieval from K9's long-term memory."""

    def __init__(self) -> None:
        super().__init__("k9_rag")

        self.declare_parameter("ollama_host", "http://127.0.0.1:11434")
        self.declare_parameter("embed_model", "qwen3-embedding:4b")
        self.declare_parameter("database_path", "~/k9_data/chroma_db")
        self.declare_parameter("collection_name", "k9_ltm_qwen3_4b")
        self.declare_parameter("default_max_results", 5)
        self.declare_parameter("min_score", 0.30)
        self.declare_parameter(
            "query_instruction",
            "Retrieve relevant passages from K9's long-term memory that help answer the user's question.",
        )

        self.ollama_host = str(self.get_parameter("ollama_host").value)
        self.embed_model = str(self.get_parameter("embed_model").value)
        self.database_path = os.path.expanduser(
            str(self.get_parameter("database_path").value)
        )
        self.collection_name = str(self.get_parameter("collection_name").value)
        self.default_max_results = int(
            self.get_parameter("default_max_results").value
        )
        self.min_score = float(self.get_parameter("min_score").value)
        self.query_instruction = str(
            self.get_parameter("query_instruction").value
        )

        os.makedirs(self.database_path, exist_ok=True)

        self.ollama_client = ollama.Client(host=self.ollama_host)
        self.chroma_client = chromadb.PersistentClient(path=self.database_path)

        # Qwen/Ollama embeddings are normalized; cosine distance therefore gives
        # a natural similarity measure for retrieval.
        self.collection = self.chroma_client.get_or_create_collection(
            name=self.collection_name,
            metadata={"hnsw:space": "cosine"},
        )

        self.service = self.create_service(
            RetrieveKnowledge,
            "/k9/rag/retrieve",
            self.retrieve_callback,
        )

        self.get_logger().info(
            f"K9 RAG ready: model={self.embed_model}, "
            f"collection={self.collection_name}, "
            f"documents={self.collection.count()}"
        )

    def _format_query(self, query: str) -> str:
        # Qwen3-Embedding recommends an instruction for retrieval queries,
        # while documents themselves are embedded without the instruction.
        return f"Instruct: {self.query_instruction}\nQuery:{query}"

    def _embed_query(self, query: str) -> List[float]:
        response = self.ollama_client.embed(
            model=self.embed_model,
            input=self._format_query(query),
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

        max_results = (
            int(request.max_results)
            if request.max_results > 0
            else self.default_max_results
        )
        max_results = max(1, max_results)

        try:
            if self.collection.count() == 0:
                response.success = True
                response.error = ""
                return response

            query_embedding = self._embed_query(query)

            results = self.collection.query(
                query_embeddings=[query_embedding],
                n_results=min(max_results, self.collection.count()),
                include=["documents", "metadatas", "distances"],
            )

            ids = (results.get("ids") or [[]])[0]
            documents = (results.get("documents") or [[]])[0]
            metadatas = (results.get("metadatas") or [[]])[0]
            distances = (results.get("distances") or [[]])[0]

            for doc_id, document, metadata, distance in zip(
                ids, documents, metadatas, distances
            ):
                score = self._similarity_from_cosine_distance(distance)

                if score < self.min_score:
                    continue

                metadata = metadata or {}

                response.ids.append(str(doc_id))
                response.documents.append(document or "")
                response.sources.append(str(metadata.get("source", "")))
                response.scores.append(float(score))
                response.metadata_json.append(
                    json.dumps(metadata, separators=(",", ":"), sort_keys=True)
                )

            response.success = True
            response.error = ""

            self.get_logger().info(
                f"RAG query returned {len(response.documents)} result(s): "
                f"{query[:80]}"
            )

        except Exception as exc:
            self.get_logger().error(f"RAG retrieval failed: {exc}")
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
