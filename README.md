# tcfm
The matlab examples used in my glorious thesis!

## Note

This was never designed to be very readable or useable by other people. If you plan to use this then I apologize in advance for the ever confusing naming and style choices.

Also, matlab is not my strongest language so if you see something very wierd (in a matlab sense) then it is probably because I tried to do something C++ in matlab.


## Dependencies 

### Robotics Toolbox 

**WARNING:** Only tested with version `9.10` [download here](http://petercorke.com/wordpress/?ddownload=383)

Known issues with versions greater than `9.10`, see https://github.com/rlober/tcfm/issues/1.

http://petercorke.com/wordpress/toolboxes/robotics-toolbox#Downloading_the_Toolbox

If you install with the zip file then you have to run `rvctools/startup_rvc.m` to put the toolbox in your matlab path. 

Make a note of where you put the toolbox: e.g. `/home/user/Documents/MATLAB/rvctools/`


### CVX
http://cvxr.com/cvx/download/

## Quick Start

1. Clone the repo somewhere.

2. Edit [line 4](https://github.com/rlober/tcfm/blob/a733ad8208bf9e83e6754672a5e413067193e485/add_compatibility_and_feasibility_path.m#L4) of `add_compatibility_and_feasibility_path.m`

```
run('../rvctools/startup_rvc.m');
```
adapt this for your install

3. Run `add_compatibility_and_feasibility_path.m`
 
4. Run `RunExampleCase.m`

You should see the time print to the dialog. 
