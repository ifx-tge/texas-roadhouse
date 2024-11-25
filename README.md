
# 4. CE238817
1. removed slider widget to gain access to P0.4 and P0.2 pins for ACTIVE PRO firmware debugging

# 3. CY8CPROTO-040T Prototyping Kit
1. P3.0, P3.2, P4.2, P4.3 pins are not brought out on headers

# 2. To initially restore project after git clone command:
When a colleague opens the application, they need to do a few things:  
1.	Run "make getlibs" or open the Library Manager and click Update to get the mtb_shared directory. 
2.	Optionally, run "make eclipse" or "make vscode" in a terminal to get the IDE-specific configuration files. They can use IAR and uVision as well, but there are more steps involved as described in the user guides for those IDEs.
3.	Open any needed Configurators and click Save to get GeneratedSource files. 
4.	And finally build the application to make sure everything works as intended.

# 1. Background
1. the following folders have been removed: /libs, /images, /build, /GeneratedSource
2. library versions have been locked
3. tools version has been locked -> export CY_TOOLS_PATHS=/opt/Tools/ModusToolbox/tools_3.3/
4. added TGE specific files to .gitignore; including /images (saves ~35MBytes)
5. minimal footprint uploaded to github


