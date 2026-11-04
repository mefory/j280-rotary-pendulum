"""GUI 入口：python main.py [--smoke 截图.png]"""
import sys

from j280wb.app import main

if __name__ == "__main__":
    args = [a for a in sys.argv[1:]]
    smoke = None
    if "--smoke" in args:
        smoke = args[args.index("--smoke") + 1]
    main(smoke)
