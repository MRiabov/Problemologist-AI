import sys
import uvicorn


def main():
    if len(sys.argv) < 2:
        print("Usage: python main.py [controller|worker-heavy|worker-light]")
        sys.exit(1)

    command = sys.argv[1]

    if command == "controller":
        uvicorn.run(
            "controller.api.main:app", host="0.0.0.0", port=8000, reload=True
        )
    elif command == "worker-heavy":
        uvicorn.run(
            "worker_heavy.app:app", host="0.0.0.0", port=8002, reload=True
        )
    elif command == "worker-light":
        uvicorn.run(
            "worker_light.app:app", host="0.0.0.0", port=8001, reload=True
        )
    else:
        print(f"Unknown command: {command}")
        print("Usage: python main.py [controller|worker-heavy|worker-light]")
        sys.exit(1)


if __name__ == "__main__":
    main()
