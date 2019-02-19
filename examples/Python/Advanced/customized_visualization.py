from open3d import dummy_callback

def dummy_func(value):
        print("inside python dummy_func:", value)
        return False

if __name__ == "__main__":
    dummy_callback(dummy_func, 123)
