within MBS;

connector KinematicPort
  // Radius vector of masscenter
  SI.Position r[3];
  // Velocity vector of masscenter
  SI.Velocity v[3];
  // Acceleration vector of masscenter
  SI.Acceleration a[3];
  // Matrix of rotation
  Real T[3, 3];
  // Vector of angular rate
  SI.AngularVelocity omega[3];
  // Vector of angular acceleration
  SI.AngularAcceleration epsilon[3];
end KinematicPort;
