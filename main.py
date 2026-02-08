import sys

import uvicorn


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "simulate":
        uvicorn.run(
            "controller.api.main:app", host="0.0.0.0", port=8000, reload=True
        )
    else:
        print("Hello from problemologist-ai!")
        print("To start the simulation server, run: python main.py simulate")


if __name__ == "__main__":
    main()
