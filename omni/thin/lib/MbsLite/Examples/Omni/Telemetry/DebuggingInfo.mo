within MbsLite.Examples.Omni.Telemetry;

model DebuggingInfo

  parameter Integer NWheels;
  parameter Integer nRollers;
  
  Real centerAltitude; 
  Real centerVerticalVelocity; 
  Real platformNonVerticalAngularVelocity; 

  Real[NWheels]    wheelAltitudes; 
  Real[NWheels]    wheelVerticalVelocities; 
  Real[NWheels]    contactNormalVelocities; 

end DebuggingInfo;

