try:
    import genesis as gs

    print(f"Has _initialized: {hasattr(gs, '_initialized')}")
    print(f"Has is_initialized: {hasattr(gs, 'is_initialized')}")
    print(f"Dir: {dir(gs)}")
except ImportError:
    print("Genesis not found")
