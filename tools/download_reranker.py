#!/usr/bin/env python3

import argparse
from pathlib import Path

from huggingface_hub import snapshot_download


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Download Qwen3-Reranker-0.6B once for offline K9 use."
    )
    parser.add_argument(
        "--output",
        default="~/k9_models/Qwen3-Reranker-0.6B",
    )
    args = parser.parse_args()

    output = Path(args.output).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)

    print(f"Downloading Qwen/Qwen3-Reranker-0.6B to {output}")

    snapshot_download(
        repo_id="Qwen/Qwen3-Reranker-0.6B",
        local_dir=str(output),
    )

    print("Download complete. Normal K9 operation can now be offline.")


if __name__ == "__main__":
    main()
