import argparse
import logging
import os
import subprocess
import time
from io import BytesIO
from pathlib import Path
from zipfile import ZipFile

import requests


def get_action_artifact(
    repo: str, py_version: str, save_dir: Path, wait_sec: int
) -> None:
    token = os.environ["GITHUB_TOKEN"]
    request_headers = {"Authorization": f"token {token}"}

    try:
        git_hash = (
            subprocess.check_output(["git", "rev-parse", "HEAD"])
            .strip()
            .decode("ascii")
        )
    except subprocess.CalledProcessError as e:
        raise RuntimeError("can't get git hash") from e

    r = requests.get(
        f"https://api.github.com/repos/{repo}/actions/artifacts",
        params={"per_page": 100},
        headers=request_headers,
    )
    if r.status_code != 200:
        raise RuntimeError(f"Can't list files ({r.status_code})")

    for artifact in r.json().get("artifacts", []):
        if (
            artifact["workflow_run"]["head_sha"] == git_hash
            and py_version in artifact["name"]
        ):
            print(f"Found artifact {artifact['name']} with {git_hash=}")

            r = requests.get(
                artifact["archive_download_url"],
                headers=request_headers,
            )
            if r.status_code != 200:
                logging.warning("Can't download artifact (%s)", r.status_code)
                return

            with ZipFile(BytesIO(r.content)) as f:
                f.extractall(path=save_dir)

            break
    else:
        logging.warning("No matching artifact found with git_hash={%s}", git_hash)

        if wait_sec > 0:
            logging.warning(
                "Trying again, remaining wait time: %d seconds", wait_sec - 30
            )
            time.sleep(30)
            get_action_artifact(repo, py_version, save_dir, wait_sec=wait_sec - 30)
        else:
            raise RuntimeError(
                f"Can't find expected {py_version} artifact with {git_hash=} "
                f"at https://api.github.com/repos/{repo}/actions/artifacts"
            )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=("Download built python wheel from GitHub Action artifacts")
    )
    parser.add_argument("repo", type=str, help="GitHub repo (e.g., haosulab/MPlib)")
    parser.add_argument(
        "--py", type=str, default="cp310", help="python version (e.g., cp310)"
    )
    parser.add_argument(
        "--save-dir",
        type=str,
        default="wheelhouse/",
        help="Directory to save downloaded artifact",
    )
    parser.add_argument(
        "--wait-sec", type=int, default="600", help="Seconds to wait for artifact"
    )
    args = parser.parse_args()

    save_dir = Path(args.save_dir)
    save_dir.mkdir()

    logging.basicConfig(
        format=(
            "[%(asctime)s] [%(name)s] [%(filename)s:%(lineno)d] "
            "[%(levelname)s] %(message)s"
        ),
        level=logging.INFO,
    )

    get_action_artifact(args.repo, args.py, save_dir, wait_sec=args.wait_sec)
