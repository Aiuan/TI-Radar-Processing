calibrateResults_dummy.mat is a dummy calibration matrix file with:
- all RxMismatch and TxMismatch values set to 0
- all peakValMat values set to the same value
- all rangeMat values set to the same value of 210

This dummy file is just a placeholder to allow the postprocessing scripts to run.
It is expected that a calibration is done and a proper calibration matrix file is passed to testList.txt.


calibrateResults_high.mat is an example calibration matrix from a particular board.
This is useful just to get an idea of what a typical calibration matrix file looks like.