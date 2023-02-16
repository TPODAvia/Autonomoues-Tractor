import os
import threading

PROTO_DIR = "./proto"


def build_protos():
    for file in os.listdir(PROTO_DIR):
        print(file)
        os.system(f"python3 -m grpc_tools.protoc -I ./proto --python_out=. --pyi_out=. --grpc_python_out=. {os.path.join(PROTO_DIR,file)}")

def run_server():
    os.system("python3 server.py")


build_protos()

