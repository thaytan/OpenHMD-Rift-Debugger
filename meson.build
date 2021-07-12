project('Godot OpenHMD visualisation plugin',
  'c', 'cpp',
  default_options: 'c_std=c99',
  version : '1.0.0',
  license : 'MIT')

dep_libm = meson.get_compiler('c').find_library('m', required: false)
deps = [
    dep_libm
]

inc = [ include_directories('submodules/godot_headers') ]
c_args = []

hmdvis_sources = [
  'src/module.c',
  'src/openhmdvisualiser.c'
]

# OpenHMD files needed for simulation
ohmd_src_dir = 'submodules/OpenHMD/src'

inc += include_directories('submodules/OpenHMD/include')
inc += include_directories('submodules/OpenHMD/src')

c_args += '-DDRIVER_OCULUS_RIFT'
c_args += '-DHAVE_OPENCV=1'

dep_threads = dependency('threads')
deps += dep_threads

dep_opencv = dependency('opencv4', required: false)
if not dep_opencv.found()
        # On Windows, search via cmake
        dep_opencv = dependency('opencv', method: 'cmake', modules: ['opencv_core'])
endif
if not dep_opencv.found()
        dep_opencv = dependency('opencv')
endif
deps += dep_opencv

hmdvis_sources += [
    'src/openhmd-compat.c',
    ohmd_src_dir / 'fusion.c',
    ohmd_src_dir / 'exponential-filter.c',
    ohmd_src_dir / 'omath.c',
    ohmd_src_dir / 'matrices.c',
    ohmd_src_dir / 'ukf.c',
    ohmd_src_dir / 'unscented.c',
    ohmd_src_dir / 'drv_oculus_rift/correspondence_search.c',
    # ohmd_src_dir / 'drv_oculus_rift/rift-sensor.c',
    ohmd_src_dir / 'drv_oculus_rift/rift-sensor-opencv.cpp',
    ohmd_src_dir / 'drv_oculus_rift/led_search.c',
    ohmd_src_dir / 'drv_oculus_rift/rift-sensor-pose-helper.c',
    ohmd_src_dir / 'drv_oculus_rift/rift-kalman-6dof.c',
    ohmd_src_dir / 'drv_oculus_rift/rift-sensor-flicker.c',
    ohmd_src_dir / 'drv_oculus_rift/rift-sensor-blobwatch.c',
    ohmd_src_dir / 'drv_oculus_rift/rift-sensor-maths.c',
]
if host_machine.system() == 'windows'
    hmdvis_sources += ohmd_src_dir / 'platform-win32.c'
else
    hmdvis_sources += ohmd_src_dir / 'platform-posix.c'
endif


hmdvis_lib = shared_library('hmdvis',
                        hmdvis_sources,
                        include_directories : inc,
                        c_args: c_args,
                        cpp_args: c_args,
                        dependencies: deps,
                        install : false)

cdata = configuration_data()
cdata.set('TOP_BUILDDIR', meson.build_root())

configure_file(input : 'src/hmdvis.gdnlib.in',
               output : 'hmdvis.gdnlib',
               configuration: cdata)

configure_file(input : 'src/OpenHMDVisualiser.gdns.in',
               output : 'OpenHMDVisualiser.gdns',
               configuration: cdata)
