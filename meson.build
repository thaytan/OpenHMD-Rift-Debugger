project('Godot OpenHMD visualisation plugin',
  'c', 'cpp',
  default_options: 'c_std=c99',
  version : '1.0.0',
  license : 'MIT')

dep_libm = meson.get_compiler('c').find_library('m', required: false)
ext_deps = [
    dep_libm
]

inc = [ include_directories('submodules/godot_headers') ]
inc += include_directories('src/lib')
c_args = []

hmdvis_sources = [
  'src/module.c',
  'src/openhmdvisualiser.c',
  'src/openhmdsensor.c',
  'src/openhmddevice.c',
]

# OpenHMD files needed for simulation
ohmd_src_dir = 'submodules/OpenHMD/src'

inc += include_directories('submodules/OpenHMD/include')
inc += include_directories('submodules/OpenHMD/src')
inc += include_directories('submodules/OpenHMD/src/drv_oculus_rift')

c_args += '-DDRIVER_OCULUS_RIFT'
c_args += '-DHAVE_OPENCV=1'

dep_threads = dependency('threads')
ext_deps += dep_threads

dep_opencv = dependency('opencv4', required: false)
if not dep_opencv.found()
        # On Windows, search via cmake
        dep_opencv = dependency('opencv', method: 'cmake', modules: ['opencv_core'])
endif
if not dep_opencv.found()
        dep_opencv = dependency('opencv')
endif
ext_deps += dep_opencv

gst_dep = dependency('gstreamer-1.0', fallback : ['gstreamer', 'gst_dep'], required: true)
gstvideo_dep = dependency('gstreamer-video-1.0', fallback: ['gst-plugins-base', 'video_dep'], required: true)
gstapp_dep = dependency('gstreamer-app-1.0', fallback: ['gst-plugins-base', 'app_dep'], required: true)

json_glib_dep = dependency('json-glib-1.0', required : false)

ext_deps += [ gst_dep, gstvideo_dep, gstapp_dep, json_glib_dep ]

trackersimlib_sources = [
  'src/lib/recording-simulator.c',
  'src/lib/recording-device-simulator.c',
  'src/lib/recording-sensor-simulator.c',
  'src/lib/recording-loader.c',
  'src/lib/recording-data.c',
  'src/lib/rift-kalman-6dof-v2.c',
  'src/lib/utils.c'
]

trackersimlib_sources += [
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
    ohmd_src_dir / 'drv_oculus_rift/rift-sensor-pose-search.c',
    ohmd_src_dir / 'drv_oculus_rift/rift-sensor-flicker.c',
    ohmd_src_dir / 'drv_oculus_rift/rift-sensor-blobwatch.c',
    ohmd_src_dir / 'drv_oculus_rift/rift-sensor-maths.c',
    # ohmd_src_dir / 'drv_oculus_rift/rift-kalman-6dof.c',
]
if host_machine.system() == 'windows'
    trackersimlib_sources += ohmd_src_dir / 'platform-win32.c'
else
    trackersimlib_sources += ohmd_src_dir / 'platform-posix.c'
endif

trackersim_lib = library('trackersimlib',
                        trackersimlib_sources,
                        include_directories : inc,
                        c_args: c_args,
                        cpp_args: c_args,
                        dependencies: ext_deps,
                        install : false)

hmdvis_lib = shared_library('hmdvis',
                        hmdvis_sources,
                        include_directories : inc,
                        c_args: c_args,
                        cpp_args: c_args,
                        dependencies: ext_deps,
                        link_with: trackersim_lib ,
                        install : false)

cdata = configuration_data()
cdata.set('TOP_BUILDDIR', meson.build_root())

configure_file(input : 'src/hmdvis.gdnlib.in',
               output : 'hmdvis.gdnlib',
               configuration: cdata)

configure_file(input : 'src/OpenHMDVisualiser.gdns.in',
               output : 'OpenHMDVisualiser.gdns',
               configuration: cdata)

dump_recording = executable('dump-recording',
                      'utils/dump-recording.c',
                      include_directories : inc,
                      c_args: c_args,
                      cpp_args: c_args,
                      dependencies: ext_deps,
                      link_with: trackersim_lib,
                      install : false)

dump_simulation = executable('dump-simulation',
                      'utils/dump-simulation.c',
                      include_directories : inc,
                      c_args: c_args,
                      cpp_args: c_args,
                      dependencies: ext_deps,
                      link_with: trackersim_lib,
                      install : false)
