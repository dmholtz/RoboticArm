# Use sphinx for documenation with python

## Initialize sphinx
```sphinx-quickstart```

## Configure sphinx
conf.py file
* conf.py must contain project name, author and version.
* conf.py must define a path to load modules from (absolute path is helpful,
    since docs is a subfolder of the projectfolder [i.e. RoboticArm])
```sys.path.insert(0, 'C:\\Users\\David\\source\\repos\\RoboticArm')```
* conf.py must load extension -> add them to array
```extensions = ['sphinx.ext.autodoc', 'sphinx.ext.coverage', 'sphinx.ext.autosummary']```
* conf.py recommended html template: sphinx_rtd_theme

index.rst file should have something like the following structure:
```
.. toctree::

.. autosummary::
   :toctree: _autosummary

   main
   :members:
   mein
   :members:

.. automodule:: main
    :members:

.. automodule:: mein
    :members:
```
Please not that "mein" and "main" are python modules, which are located
in the directory specified in conf.py

Apart from mentioning all the relevent modules in the autosummary tab, it is
necessary to specify the moduls.

Please not that sphinx runs each module as it builds the html-doc. Thus, moduls
containing executable code should not be contained in the API doc.

## Generate documentation
```sphinx-apidoc -o . ..```
```make html```