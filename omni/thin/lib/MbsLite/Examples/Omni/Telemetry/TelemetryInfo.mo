within MbsLite.Examples.Omni.Telemetry;

model TelemetryInfo

  parameter Integer NWheels;
  parameter Integer nRollers;
  
  Real kineticEnergy             (stateSelect = StateSelect.never); 
  Real centerVelocityNorm        (stateSelect = StateSelect.never); 
  Real platformAngularVelocity   (stateSelect = StateSelect.never); 

  Integer[NWheels] indicesOfRollerInContact     (each stateSelect = StateSelect.never); 
  // note that these can become zero if the repective vectors and wheel axes become orthogonal
  Real[NWheels]    contactPointVelocitiesSigned (each stateSelect = StateSelect.never); 
  Real[NWheels]    frictionNormsSigned          (each stateSelect = StateSelect.never); 
  Real[NWheels]    normalReactions              (each stateSelect = StateSelect.never); 

  LagrangianCoords lagrangianCoords
    ( NWheels  = NWheels
    , nRollers = nRollers
    ); 
  DebuggingInfo    debuggingInfo
    ( NWheels  = NWheels
    , nRollers = nRollers
    ); 

end TelemetryInfo;

