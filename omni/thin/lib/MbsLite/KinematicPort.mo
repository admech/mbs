within MbsLite;

connector KinematicPort

  SI.Position[3]             r        "Radius vector of masscenter";
  SI.Velocity[3]             v        "Velocity vector of masscenter";
  SI.Acceleration[3]         a        "Acceleration vector of masscenter";
  Real[3, 3]                 T        "Matrix of rotation";
  SI.AngularVelocity[3]      omega    "Vector of angular rate";
  SI.AngularAcceleration[3]  epsilon  "Vector of angular acceleration";

end KinematicPort;
