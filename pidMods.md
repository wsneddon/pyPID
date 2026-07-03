Would like to create a python PID block that is a fork of https://pypi.org/project/simple-pid/

First read the docs of above project.
Also try to find way to clone this simple-pid to local drive

Would like to modify the base in the following way.

1. Add a manual mode and auto mode also cascade mode where it accepts a remote setpoint
   should be able to change during operation.
   a. manual mode out puts stay at last state but is writable form outside code.
   b. auto mode output calculated not written to by outside code.
   c. cascade same as auto but uses rsp instead of sp for calculation.
2. add a way to get reverse acting behavior by parameter. 
	a. would like to change behavior instead of negating the gain but by changing error definiton to from (SP - PV)
	   to PV-SP
3. implement optional output clamping to prevent integral wind-up if it does not exist.
4. have a bias term which is identical to last output.
     would like this term exposed to where it can be written to by other code. Should alow outside code to written then overwitten in previous scans.
5. bias rights not allowed in manual.  output to overwrite bias on transition from manual to auto
6. Would also like to concider adding engineering units and scaling to the input.
7. Would also like to concider adding alarming add alarm setpoints inputs and boolean outputs for the following states.
	LLL  PV less than low low low setpoint lllsp
	LL  PV less than low low setpoint llsp
	L  PV less than low setpoint lsp
	H  PV greater than high setpoint hsp
	HH  PV greater than  high high setpoint hhsp
	HHH  PV greater than  high high high setpoint hhhsp
    YelDev abs(SP -PV) > yellow deviation setpoint yeldevSP
    orgDev abs(SP -PV) > orange deviation setpoint orgdevSP
8. Concider adding sample time and internal scheduler that execute the algoythm when in auto mode without external call.

Would also like a simulation module which is decoupled from the pid object but can easily be connected to pid.  it's input should be the loop output. and it's output should be able to connect to the loop input.  The block should impliment a FOLPD algorythm.

