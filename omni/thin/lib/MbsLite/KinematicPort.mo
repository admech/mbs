within MbsLite;

connector KinematicPort

  SI.Position[3]             r        (each stateSelect = StateSelect.never) "Radius vector of masscenter";
  SI.Velocity[3]             v        (each stateSelect = StateSelect.never) "Velocity vector of masscenter";
  SI.Acceleration[3]         a        (each stateSelect = StateSelect.never) "Acceleration vector of masscenter";
  Real[3, 3]                 T        (each stateSelect = StateSelect.never) "Matrix of rotation";
  SI.AngularVelocity[3]      omega    (each stateSelect = StateSelect.never) "Vector of angular rate";
  SI.AngularAcceleration[3]  epsilon  (each stateSelect = StateSelect.never) "Vector of angular acceleration";

end KinematicPort;
