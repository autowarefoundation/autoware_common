from rpyutils import import_c_library
package = 'lanelet2_extension_python'

lanelet2_extension_utility_implementation = import_c_library('._lanelet2_extension_python_boost_python_utility', package)
lanelet2_extension_projection_implementation = import_c_library('._lanelet2_extension_python_boost_python_projection', package)
