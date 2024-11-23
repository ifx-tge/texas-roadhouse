
#2. To initially restore project after git clone command:
When a colleague opens the application, they need to do a few things: 
1.	Run "make eclipse" or "make vscode" in a terminal to get the IDE-specific configuration files. They can use IAR and uVision as well, but there are more steps involved as described in the user guides for those IDEs. 
2.	Then, run "make getlibs" or open the Library Manager and click Update to get the mtb_shared directory. 
3.	Open any needed Configurators and click Save to get GeneratedSource files. 
4.	And finally build the application to make sure everything works as intended.


#1. Background
- the following folders have been removed: /libs, /images, /build, /GeneratedSource
- library versions have been locked
- tools version has been locked -> export CY_TOOLS_PATHS=/opt/Tools/ModusToolbox/tools_3.3/
- added TGE specific files to .gitignore; including /images (saves ~35MBytes)
- minimal footprint uploaded to github


