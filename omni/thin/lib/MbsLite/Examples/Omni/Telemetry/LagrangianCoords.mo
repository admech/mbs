within MbsLite.Examples.Omni.Telemetry;

model LagrangianCoords

  parameter Integer NWheels;
  parameter Integer nRollers;
  
  Real x     (stateSelect = StateSelect.never) "mass center x"; 
  Real y     (stateSelect = StateSelect.never) "mass center y";
  Real theta (stateSelect = StateSelect.never) "angle of self-rotation about the Z axis, CCW+ looking from the end of Z";

  Real[NWheels]           chi   (each stateSelect = StateSelect.never) "wheel  self-rotation angles CCW+ looking from outside the vehicle";
  Real[NWheels, nRollers] phi   (each stateSelect = StateSelect.never) "roller self-rotation angles CCW+ looking from the end of roller axis";
  Real[NWheels, nRollers] Dphi  (each stateSelect = StateSelect.never) "roller self-rotation angular velocities CCW+ looking from the end of roller axis";

end LagrangianCoords;
