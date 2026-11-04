"""J280 上位机工作台。

协议与设备内核直接复用本项目的 `host/` 包（单一事实来源，禁止复制协议实现），
本包只做三件事：共享设备桥（bridge）、实物模型动画（model3d/modelview）、
Agent JSON 接口（agent/server/cli）与 GUI 组装（app/main）。

内核位置解析顺序（先命中先用）：

1. 环境变量 ``J280_CORE`` —— 指向外部内核根目录（即含 ``host/`` 的目录）
2. ``j280_core.txt`` —— 上位机目录或工作区根目录下的一行纯文本路径；
   相对路径按该文件所在目录解析。团队各自指定内核位置时用这个
3. 默认当前 ``上位机/`` 项目内置内核

换外部内核不需要改代码。板标识与能力位由 ``host/protocol.py`` 解析内核里的
``rtl/j280_debug_params.vh`` 得到，因此 RTL 改了 host 侧会自动跟上。
"""
import os
import sys
from pathlib import Path

_PACKAGE = Path(__file__).resolve().parent      # 上位机/j280wb
_APP = _PACKAGE.parent                          # 上位机

BACKENDS = ("模拟", "锁步HIL", "实板", "回放")
AGENT_DEFAULT_PORT = 28080


def _candidates():
    """按优先级产出 (内核根候选, 来源说明)。"""
    env = os.environ.get("J280_CORE", "").strip()
    if env:
        yield Path(env), "环境变量 J280_CORE"
    for base in (_APP, _APP.parent):
        config = base / "j280_core.txt"
        if not config.is_file():
            continue
        text = config.read_text(encoding="utf-8").strip()
        if not text or text.startswith("#"):
            continue
        path = Path(text)
        yield (path if path.is_absolute() else config.parent / path), str(config)
    # 上位机项目自带 host/config/rtl，默认不再依赖旁边的 DE10 工程。
    yield _APP, f"默认（上位机内置内核：{_APP}）"
    # 保留外部内核作为迁移/回归覆盖，不影响主项目的独立运行。
    fallback = _APP.parent / "DE10"
    yield fallback, f"兼容回退（{fallback}）"


def core_root():
    """返回内核仓库根目录（含 host/ 包的目录），都找不到时给出可操作的报错。"""
    tried = []
    for path, source in _candidates():
        path = path.resolve()
        if (path / "host" / "__init__.py").is_file() and (path / "config" / "parameters.json").is_file():
            return path
        tried.append(f"    {path}   ← {source}")
    raise ImportError(
        "找不到 J280 内核仓库（需同时存在 host/__init__.py 与 config/parameters.json）。\n"
        "已尝试：\n" + "\n".join(tried) + "\n"
        "解决办法：设置环境变量 J280_CORE 指向内核仓库根目录，"
        "或在本目录/工作区根目录写一行路径到 j280_core.txt。")


_CORE = core_root()
if str(_CORE) not in sys.path:
    sys.path.insert(0, str(_CORE))

CORE_ROOT = str(_CORE)
