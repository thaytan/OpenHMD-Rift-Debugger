project('Godot OpenHMD visualisation plugin', 'c',
  version : '1.0.0',
  license : 'MIT')

inc = [ include_directories('subprojects/godot_headers') ]
inc += include_directories('subprojects/OpenHMD')

hmdvis_sources = [
  'src/module.c',
  'src/openhmdvisualiser.c'
]

hmdvis_lib = shared_library('hmdvis',
                        hmdvis_sources,
                        include_directories : inc,
                        install : false)

cdata = configuration_data()
cdata.set('TOP_BUILDDIR', meson.build_root())

configure_file(input : 'src/hmdvis.gdnlib.in',
               output : 'hmdvis.gdnlib',
               configuration: cdata)

configure_file(input : 'src/OpenHMDVisualiser.gdns.in',
               output : 'OpenHMDVisualiser.gdns',
               configuration: cdata)
