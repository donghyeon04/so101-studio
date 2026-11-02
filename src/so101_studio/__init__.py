import os

# shortcut: torch(libiomp5)와 conda placo(libomp)가 OpenMP 런타임을 둘 다 올려 Windows에서 종료된다.
# 수치 결과가 어긋날 수 있는 우회책이라 FK는 실측(T3)과 단위 테스트로 검증한다. 해소되면 제거.
os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")
