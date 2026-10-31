"""下载固定版本 go2_rl_lab 模型、部署资源和许可，并校验每个文件。

项目根目录运行：.venv/bin/python scripts/fetch_go2_policy.py
"""

import hashlib
import json
import tempfile
import time
from pathlib import Path
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]


def main():
    c = json.loads((ROOT / "configs/go2_controller.json").read_text())
    lock = json.loads((ROOT / "configs/go2_rl_lab.lock.json").read_text())
    for item in lock["files"]:
        destination = (ROOT / item["path"]).resolve()
        destination.relative_to(ROOT / "third_party/go2_rl_lab")
        url, checksum = item["url"], item["sha256"]
        name = str(destination.relative_to(ROOT))
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.exists():
            if hashlib.sha256(destination.read_bytes()).hexdigest() != checksum:
                raise RuntimeError(f"{destination} 已存在但内容不同，保留原文件；请先核查")
            print(f"已校验 {name}")
            continue
        for attempt in range(3):
            try:
                with urlopen(url, timeout=60) as response:
                    payload = response.read()
                break
            except OSError:
                if attempt == 2:
                    raise
                time.sleep(1)
        if hashlib.sha256(payload).hexdigest() != checksum:
            raise RuntimeError(f"{name} 下载内容与固定 SHA256 不符，未写入")
        with tempfile.NamedTemporaryFile(dir=destination.parent, delete=False) as temp:
            temp.write(payload)
            temp_path = Path(temp.name)
        temp_path.replace(destination)
        print(f"已下载并校验 {name}")
    # 在下载目录同时保留来源与许可状态，避免只拷贝模型后丢失说明。
    folder = ROOT / "third_party/go2_rl_lab"
    (folder / "SOURCE.json").write_text(json.dumps(c, indent=2, ensure_ascii=False) + "\n")


if __name__ == "__main__":
    main()
