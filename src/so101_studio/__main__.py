import sys

if any(x in ("doctor", "find-port", "incident") for x in sys.argv[1:]):  # lerobot 없이도 돈다: 무거운 import를 하지 않는다
    from .bringup import main
else:
    from .cli import main

main()
