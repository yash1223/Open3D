import open3d as _open3d_internal
import functools


def add(a, b):
    return a + b


# def wrap_read_point_cloud():
#     def read_point_cloud(filename, format='auto'):
#         """
#         Read geometry::LineSet from file
#         :param filename: File path
#         :param format
#         :return: open3d.geometry.PointCloud
#         """
#         return _open3d_internal.read_point_cloud(filename, format=format)
#     read_point_cloud.__doc__ = read_point_cloud.__doc__ + _open3d_internal.read_point_cloud.__doc__
#     return read_point_cloud
#
# read_point_cloud = wrap_read_point_cloud()
#
# print(read_point_cloud.__doc__)
#
# a = _open3d_internal.read_point_cloud
# b = 1


def define_doc_param(param_name, param_doc):
    def define_doc_param_decorator(func):
        @functools.wraps(func)
        def wrapped_func(*args, **kwargs):
            return func(args, **kwargs)
        if wrapped_func.__doc__ is None:
            wrapped_func.__doc__ = ""
        wrapped_func.__doc__ += f"{param_name}: {param_doc}" + "\n"
        return wrapped_func
    return define_doc_param_decorator

@define_doc_param("filename", "Input path")
@define_doc_param("format", "Format")
def read_point_cloud(filename, format='auto'):
    return _open3d_internal.read_point_cloud(filename, format=format)


print(read_point_cloud.__doc__)
