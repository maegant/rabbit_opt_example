# rabbit_opt_example
This repository is intended to provide a tutorial on how to set up a (P)HZD gait generation environment for the rabbit bipedal robot.

<!-- # Changes required to frost-dev
1. Remove '2d' from validateattributes in HolonomicConstraint.m (inside system/@HolonomicConstraint class) (line 363) -->

# Tutorial 
1. The most recent version of FROST requires MATLAB2022b. To avoid this you can try pulling FROST repository commits prior to the one listed in step 3. It also may require ubuntu >18.04. 
2. Install FROST following the installation instructions provided [here](https://ayonga.github.io/frost-dev/pages/installation.html)
3. This tutorial assumes that the FROST repository is set to the master branch with the following commit
    ```
    git checkout master
    git cherry-pick 5e17cf8d9d2f2c239c3589d0a8bdb8ea49f09d53
    ```
- Note that you may need to add the following line to your .bashrc in order to successfully use the later FROST commits
    ```
    LD_LIBRARY_PATH=/usr/local/Wolfram/Mathematica/12.0/SystemFiles/Links/WSTP/DeveloperKit/Linux-x86-64/CompilerAdditions:$LD_LIBRARY_PATH
    ```
4. Edit the `frost_path` variable in main_opt (line 23) to point to your frost-dev cloned repo 
5. The first time you run the main_opt script, you will need to compile the .mex files required to run the gait generation problem. To set the compile flags to true, change lines 39 and 48 of main_opt to:
    ```
    compileMex = [1,1];
    do_compile = [1,1,1];
    ```
    After compiling the mex files, you can set these lines to false (0). Note that you will need to recompile the .mex files if you change the gait generation problem setup.
6. Run the entire `main_opt.m` script to generate and simulate a gait