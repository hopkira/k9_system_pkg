\
#!/usr/bin/env python3
"""
Build or update K9's Chroma long-term-memory index.

Documents are embedded WITHOUT the Qwen retrieval instruction.
At runtime, rag.py adds the instruction to the query only.
"""

import argparse
import csv
import hashlib
import os
from pathlib import Path
from typing import Dict, List, Tuple

import chromadb
import ollama


DEFAULT_MODEL = "qwen3-embedding:4b"
DEFAULT_COLLECTION = "k9_ltm_qwen3_4b"
DEFAULT_DB = "~/k9_data/chroma_db"


def deterministic_id(source: str, row_number: int, text: str) -> str:
    digest = hashlib.sha256(
        f"{source}:{row_number}:{text}".encode("utf-8")
    ).hexdigest()[:24]
    return f"k9_{digest}"


def read_csv_documents(
    csv_path: Path,
    text_column: str,
    title_column: str | None,
) -> Tuple[List[str], List[str], List[Dict]]:
    documents: List[str] = []
    ids: List[str] = []
    metadatas: List[Dict] = []

    with csv_path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)

        if not reader.fieldnames or text_column not in reader.fieldnames:
            raise ValueError(
                f"Column '{text_column}' not found. "
                f"Available columns: {reader.fieldnames}"
            )

        if title_column and title_column not in reader.fieldnames:
            raise ValueError(
                f"Title column '{title_column}' not found. "
                f"Available columns: {reader.fieldnames}"
            )

        for row_number, row in enumerate(reader, start=1):
            text = (row.get(text_column) or "").strip()
            if not text:
                continue

            metadata = {
                "source": csv_path.name,
                "row": row_number,
                "type": "story",
            }

            if title_column:
                title = (row.get(title_column) or "").strip()
                if title:
                    metadata["title"] = title

            documents.append(text)
            ids.append(deterministic_id(csv_path.name, row_number, text))
            metadatas.append(metadata)

    return documents, ids, metadatas


def batches(items: List, size: int):
    for start in range(0, len(items), size):
        yield start, items[start:start + size]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True, help="CSV file to ingest")
    parser.add_argument("--column", default="synopsis", help="Text column")
    parser.add_argument("--title-column", default=None)
    parser.add_argument("--db", default=DEFAULT_DB)
    parser.add_argument("--collection", default=DEFAULT_COLLECTION)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--ollama-host", default="http://127.0.0.1:11434")
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument(
        "--reset",
        action="store_true",
        help="Delete and rebuild this collection before indexing",
    )
    args = parser.parse_args()

    csv_path = Path(args.csv).expanduser().resolve()
    db_path = os.path.expanduser(args.db)

    documents, ids, metadatas = read_csv_documents(
        csv_path,
        args.column,
        args.title_column,
    )

    if not documents:
        raise RuntimeError("No non-empty documents found")

    client = chromadb.PersistentClient(path=db_path)

    if args.reset:
        try:
            client.delete_collection(args.collection)
            print(f"Deleted existing collection: {args.collection}")
        except Exception:
            pass

    collection = client.get_or_create_collection(
        name=args.collection,
        metadata={"hnsw:space": "cosine"},
    )

    ollama_client = ollama.Client(host=args.ollama_host)

    print(f"Input:       {csv_path}")
    print(f"Documents:   {len(documents)}")
    print(f"Model:       {args.model}")
    print(f"Collection:  {args.collection}")
    print(f"Database:    {db_path}")

    for start, document_batch in batches(documents, args.batch_size):
        end = start + len(document_batch)

        result = ollama_client.embed(
            model=args.model,
            input=document_batch,
        )

        embeddings = getattr(result, "embeddings", None)
        if embeddings is None:
            embeddings = result["embeddings"]

        collection.upsert(
            ids=ids[start:end],
            embeddings=embeddings,
            documents=document_batch,
            metadatas=metadatas[start:end],
        )

        print(f"Indexed {end}/{len(documents)}")

    print(
        f"Complete. Collection '{args.collection}' now contains "
        f"{collection.count()} document(s)."
    )


if __name__ == "__main__":
    main()
