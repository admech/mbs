within MbsLite.Examples.OmniVehicle;

record LagrangianCoords

  parameter Params  params;
  parameter Integer totalNumRollers = params.NWheels * params.nRollers;
  
  Real                  x      "mass center x"; 
  Real                  y      "mass center y";
  Real                  theta  "angle of self-rotation about the Z axis, CCW+ looking from the end of Z";

  Real[params.NWheels]  chi    "wheel  self-rotation angles CCW+ looking from outside the vehicle";
  Real[totalNumRollers] phi    "roller self-rotation angles CCW+ looking from the end of roller axis";

end LagrangianCoords;
