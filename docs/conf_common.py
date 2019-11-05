# -*- coding: utf-8 -*-
#
# Common (non-language-specific) configuration for Read The Docs & Sphinx
#
# Based on a Read the Docs Template documentation build configuration file,
# created by sphinx-quickstart on Tue Aug 26 14:19:49 2014.
#
# This file is imported from a language-specific conf.py (ie en/conf.py or
# zh_CN/conf.py)
#
# Note that not all possible configuration values are present in this
# autogenerated file.
#
# All configuration values have a default; values that are commented out
# serve to show the default.

from __future__ import print_function
from __future__ import unicode_literals
import sys
import os
import re
import subprocess

# Note: If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute

from local_util import run_cmd_get_output, copy_if_modified

# build_docs on the CI server sometimes fails under Python3. This is a workaround:
sys.setrecursionlimit(3500)

try:
    builddir = os.environ['BUILDDIR']
except KeyError:
    builddir = '_build'

# Fill in a default IDF_PATH if it's missing (ie when Read The Docs is building the docs)
try:
    idf_path = os.environ['IDF_PATH']
except KeyError:
    idf_path = os.path.realpath(os.path.join(os.path.dirname(__file__), '..'))


def call_with_python(cmd):
    # using sys.executable ensures that the scripts are called with the same Python interpreter
    if os.system('{} {}'.format(sys.executable, cmd)) != 0:
        raise RuntimeError('{} failed'.format(cmd))


# Call Doxygen to get XML files from the header files
print("Calling Doxygen to generate latest XML files")
if os.system("doxygen ../Doxyfile") != 0:
    raise RuntimeError('Doxygen call failed')

# Doxygen has generated XML files in 'xml' directory.
# Copy them to 'xml_in', only touching the files which have changed.
copy_if_modified('xml/', 'xml_in/')

# Generate 'api_name.inc' files using the XML files by Doxygen
call_with_python('../gen-dxd.py')


def find_component_files(parent_dir, target_filename):
    parent_dir = os.path.abspath(parent_dir)
    result = []

    component_files = dict()

    for (dirpath, dirnames, filenames) in os.walk(parent_dir):
        try:
            # note: trimming "examples" dir as MQTT submodule
            # has its own examples directory in the submodule, not part of IDF
            dirnames.remove("examples")
        except ValueError:
            pass
        if target_filename in filenames:
            component_files[os.path.basename(dirpath)] = os.path.join(dirpath, target_filename)

    components = sorted(component_files.keys())

    for component in components:
        result.append(component_files[component])

    print("List of %s: %s" % (target_filename, ", ".join(components)))
    return result


# Generate 'kconfig.inc' file from components' Kconfig files
print("Generating kconfig.inc from kconfig contents")
kconfig_inc_path = '{}/inc/kconfig.inc'.format(builddir)
temp_sdkconfig_path = '{}/sdkconfig.tmp'.format(builddir)

kconfigs = find_component_files("../../components", "Kconfig")
kconfig_projbuilds = find_component_files("../../components", "Kconfig.projbuild")
sdkconfig_renames = find_component_files("../../components", "sdkconfig.rename")

# trim the esp32s2beta component, until we have proper multi-target support
kconfigs = [k for k in kconfigs if "esp32s2beta" not in k]
kconfig_projbuilds = [k for k in kconfig_projbuilds if "esp32s2beta" not in k]
sdkconfig_renames = [r for r in sdkconfig_renames if "esp32s2beta" not in r]

confgen_args = [sys.executable,
                "../../tools/kconfig_new/confgen.py",
                "--kconfig", "../../Kconfig",
                "--sdkconfig-rename", "../../sdkconfig.rename",
                "--config", temp_sdkconfig_path,
                "--env", "COMPONENT_KCONFIGS={}".format(" ".join(kconfigs)),
                "--env", "COMPONENT_KCONFIGS_PROJBUILD={}".format(" ".join(kconfig_projbuilds)),
                "--env", "COMPONENT_SDKCONFIG_RENAMES={}".format(" ".join(sdkconfig_renames)),
                "--env", "IDF_PATH={}".format(idf_path),
                "--output", "docs", kconfig_inc_path + '.in'
                ]
subprocess.check_call(confgen_args)
copy_if_modified(kconfig_inc_path + '.in', kconfig_inc_path)

# Generate 'esp_err_defs.inc' file with ESP_ERR_ error code definitions
esp_err_inc_path = '{}/inc/esp_err_defs.inc'.format(builddir)
call_with_python('../../tools/gen_esp_err_to_name.py --rst_output ' + esp_err_inc_path + '.in')
copy_if_modified(esp_err_inc_path + '.in', esp_err_inc_path)


# Generate version-related includes
#
# (Note: this is in a function as it needs to access configuration to get the language)
def generate_version_specific_includes(app):
    print("Generating version-specific includes...")
    version_tmpdir = '{}/version_inc'.format(builddir)
    call_with_python('../gen-version-specific-includes.py {} {}'.format(app.config.language, version_tmpdir))
    copy_if_modified(version_tmpdir, '{}/inc'.format(builddir))


# Generate toolchain download links
print("Generating toolchain download links")
base_url = 'https://dl.espressif.com/dl/'
toolchain_tmpdir = '{}/toolchain_inc'.format(builddir)
call_with_python('../gen-toolchain-links.py ../../tools/toolchain_versions.mk {} {}'.format(base_url, toolchain_tmpdir))
copy_if_modified(toolchain_tmpdir, '{}/inc'.format(builddir))

# http://stackoverflow.com/questions/12772927/specifying-an-online-image-in-sphinx-restructuredtext-format
#
suppress_warnings = ['image.nonlocal_uri']

# -- General configuration ------------------------------------------------


# If your documentation needs a minimal Sphinx version, state it here.
# needs_sphinx = '1.0'

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = ['breathe',
              'link-roles',
              'sphinxcontrib.blockdiag',
              'sphinxcontrib.seqdiag',
              'sphinxcontrib.actdiag',
              'sphinxcontrib.nwdiag',
              'sphinxcontrib.rackdiag',
              'sphinxcontrib.packetdiag',
              'html_redirects',
              'sphinx.ext.todo',
              ]

# sphinx.ext.todo extension parameters
# If the below parameter is True, the extension
# produces output, else it produces nothing.
todo_include_todos = False

# Enabling this fixes cropping of blockdiag edge labels
seqdiag_antialias = True

# Breathe extension variables

# Doxygen regenerates files in 'xml/' directory every time,
# but we copy files to 'xml_in/' only when they change, to speed up
# incremental builds.
breathe_projects = {"esp32-idf": "xml_in/"}
breathe_default_project = "esp32-idf"

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# The suffix of source filenames.
source_suffix = ['.rst', '.md']

source_parsers = {'.md': 'recommonmark.parser.CommonMarkParser',
                  }

# The encoding of source files.
# source_encoding = 'utf-8-sig'

# The master toctree document.
master_doc = 'index'


# The version info for the project you're documenting, acts as replacement for
# |version| and |release|, also used in various other places throughout the
# built documents.
#

# Readthedocs largely ignores 'version' and 'release', and displays one of
# 'latest', tag name, or branch name, depending on the build type.
# Still, this is useful for non-RTD builds.
# This is supposed to be "the short X.Y version", but it's the only version
# visible when you open index.html.
# Display full version to make things less confusing.
version = run_cmd_get_output('git describe')
# The full version, including alpha/beta/rc tags.
# If needed, nearest tag is returned by 'git describe --abbrev=0'.
release = version
print('Version: {0}  Release: {1}'.format(version, release))

# There are two options for replacing |today|: either, you set today to some
# non-false value, then it is used:
# today = ''
# Else, today_fmt is used as the format for a strftime call.
# today_fmt = '%B %d, %Y'

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
exclude_patterns = ['_build','README.md']

# The reST default role (used for this markup: `text`) to use for all
# documents.
# default_role = None

# If true, '()' will be appended to :func: etc. cross-reference text.
# add_function_parentheses = True

# If true, the current module name will be prepended to all description
# unit titles (such as .. function::).
# add_module_names = True

# If true, sectionauthor and moduleauthor directives will be shown in the
# output. They are ignored by default.
# show_authors = False

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = 'sphinx'

# A list of ignored prefixes for module index sorting.
# modindex_common_prefix = []

# If true, keep warnings as "system message" paragraphs in the built documents.
# keep_warnings = False


# -- Options for HTML output ----------------------------------------------

# Custom added feature to allow redirecting old URLs
#
# Redirects should be listed in page_redirects.xt
#
with open("../page_redirects.txt") as f:
    lines = [re.sub(" +", " ", l.strip()) for l in f.readlines() if l.strip() != "" and not l.startswith("#")]
    for line in lines:  # check for well-formed entries
        if len(line.split(' ')) != 2:
            raise RuntimeError("Invalid line in page_redirects.txt: %s" % line)
html_redirect_pages = [tuple(l.split(' ')) for l in lines]

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
html_theme = 'sphinx_rtd_theme'

# Theme options are theme-specific and customize the look and feel of a theme
# further.  For a list of options available for each theme, see the
# documentation.
# html_theme_options = {}

# Add any paths that contain custom themes here, relative to this directory.
# html_theme_path = []

# The name for this set of Sphinx documents.  If None, it defaults to
# "<project> v<release> documentation".
# html_title = None

# A shorter title for the navigation bar.  Default is the same as html_title.
# html_short_title = None

# The name of an image file (relative to this directory) to place at the top
# of the sidebar.
html_logo = "../_static/espressif-logo.svg"

# The name of an image file (within the static path) to use as favicon of the
# docs.  This file should be a Windows icon file (.ico) being 16x16 or 32x32
# pixels large.
# html_favicon = None

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['../_static']

# Add any extra paths that contain custom files (such as robots.txt or
# .htaccess) here, relative to this directory. These files are copied
# directly to the root of the documentation.
# html_extra_path = []

# If not '', a 'Last updated on:' timestamp is inserted at every page bottom,
# using the given strftime format.
# html_last_updated_fmt = '%b %d, %Y'

# If true, SmartyPants will be used to convert quotes and dashes to
# typographically correct entities.
# html_use_smartypants = True

# Custom sidebar templates, maps document names to template names.
# html_sidebars = {}

# Additional templates that should be rendered to pages, maps page names to
# template names.
# html_additional_pages = {}

# If false, no module index is generated.
# html_domain_indices = True

# If false, no index is generated.
# html_use_index = True

# If true, the index is split into individual pages for each letter.
# html_split_index = False

# If true, links to the reST sources are added to the pages.
# html_show_sourcelink = True

# If true, "Created using Sphinx" is shown in the HTML footer. Default is True.
# html_show_sphinx = True

# If true, "(C) Copyright ..." is shown in the HTML footer. Default is True.
# html_show_copyright = True

# If true, an OpenSearch description file will be output, and all pages will
# contain a <link> tag referring to it.  The value of this option must be the
# base URL from which the finished HTML is served.
# html_use_opensearch = ''

# This is the file name suffix for HTML files (e.g. ".xhtml").
# html_file_suffix = None

# Output file base name for HTML help builder.
htmlhelp_basename = 'ReadtheDocsTemplatedoc'


# -- Options for LaTeX output ---------------------------------------------

latex_elements = {
    # The paper size ('letterpaper' or 'a4paper').
    # 'papersize': 'letterpaper',
    #
    # The font size ('10pt', '11pt' or '12pt').
    # 'pointsize': '10pt',
    #
    # Additional stuff for the LaTeX preamble.
    # 'preamble': '',
}

# Grouping the document tree into LaTeX files. List of tuples
# (source start file, target name, title,
#  author, documentclass [howto, manual, or own class]).
latex_documents = [
    ('index', 'ReadtheDocsTemplate.tex', u'Read the Docs Template Documentation',
     u'Read the Docs', 'manual'),
]

# The name of an image file (relative to this directory) to place at the top of
# the title page.
# latex_logo = None

# For "manual" documents, if this is true, then toplevel headings are parts,
# not chapters.
# latex_use_parts = False

# If true, show page references after internal links.
# latex_show_pagerefs = False

# If true, show URL addresses after external links.
# latex_show_urls = False

# Documents to append as an appendix to all manuals.
# latex_appendices = []

# If false, no module index is generated.
# latex_domain_indices = True


# -- Options for manual page output ---------------------------------------

# One entry per manual page. List of tuples
# (source start file, name, description, authors, manual section).
man_pages = [
    ('index', 'readthedocstemplate', u'Read the Docs Template Documentation',
     [u'Read the Docs'], 1)
]

# If true, show URL addresses after external links.
# man_show_urls = False


# -- Options for Texinfo output -------------------------------------------

# Grouping the document tree into Texinfo files. List of tuples
# (source start file, target name, title, author,
#  dir menu entry, description, category)
texinfo_documents = [
    ('index', 'ReadtheDocsTemplate', u'Read the Docs Template Documentation',
     u'Read the Docs', 'ReadtheDocsTemplate', 'One line description of project.',
     'Miscellaneous'),
]

# Documents to append as an appendix to all manuals.
# texinfo_appendices = []

# If false, no module index is generated.
# texinfo_domain_indices = True

# How to display URL addresses: 'footnote', 'no', or 'inline'.
# texinfo_show_urls = 'footnote'

# If true, do not generate a @detailmenu in the "Top" node's menu.
# texinfo_no_detailmenu = False


# Override RTD CSS theme to introduce the theme corrections
# https://github.com/rtfd/sphinx_rtd_theme/pull/432
def setup(app):
    app.add_stylesheet('theme_overrides.css')
    generate_version_specific_includes(app)
