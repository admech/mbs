within MBS.Examples.OmniVehicle;

partial model RollerContactTrackingGeneralStep
  extends Constraint;
  parameter Integer n = 4 "Number of rollers";
  parameter Real R = 1 "Omni wheel outer radius (ellipse small axis)";
  parameter Real psi = 0 "Angle of roller distortion (fixed axis turn)";
  parameter Real cospsi = cos(psi) "Auxiliary value";

  parameter Real alpha = Modelica.Constants.pi/n
    "Max angle of the half-sector";
  parameter Real Q = R/cos(psi) "Ellipse large axis";
  parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
  parameter Real L1 = R*sin(alpha)
    "Half roller visible (projection) length";
  parameter Real L2 = L1/cos(psi) "Half roller length";
  parameter Real[3] nA = {0, 1, 0}
    "Roller over horizontal surface: vertical unit vector (in global coordinates)";
  parameter Real[2] gradAtRollerTip = 2*{L2/Q^2, R1/R^2};
  parameter Real[2] n_at_max = gradAtRollerTip/sqrt(gradAtRollerTip*gradAtRollerTip)
    "Normal to ellipse in local coord (it is in vert plane)";
  parameter Real[3] i = {1, 0, 0} "Roller axis of symmetry unit vector";
  parameter Real cos_of_max = 0.3820515; // For psi = 1.0
//  parameter Real cos_of_max = 0.44; // For psi = 0.9
//  parameter Real cos_of_max = 0.4927; // For psi = 0.8
//  parameter Real cos_of_max = 0.5408; // For psi = 0.7
//  parameter Real cos_of_max = 0.584; // For psi = 0.6
//  parameter Real cos_of_max = 0.62; // For psi = 0.5
//  parameter Real cos_of_max = 0.65128; // For psi = 0.4
//  parameter Real cos_of_max = n_at_max * {1, 0};
//  parameter Real cos_of_max = cos(Modelica.Constants.pi/2 - alpha); // cos(pi/2 - pi/n)
  SI.Position[3] rA; // A - plane
  SI.Position[3] rB; // B - roller

// Delivered from above
  Real[3] rO "Wheel center position";
  Real[3] n1k "Horizontal wheel axis";

// Auxiliaries
  Real[3] ni "Roller axis in global coords";
  Real[3] rho "Unit vector between wheel and roller mass centers";
  Real[3] OBPB; // Contact poit relative position
  Real h;     // point of contact vertical coordinate
  Real j;
  Real cosBtwAxisAndVert;
  Real w;
  Real cosq;
  Real sinq;
equation
  w = abs(cosBtwAxisAndVert);

// Intermediate roller coordinates:
  ni = InPortB.T*i;
  rho = (rO - InPortB.r)/R1; // to verify unit length
  cosq = rho*nA;
  sinq = cross(nA, rho)*n1k;

  cosBtwAxisAndVert = ni*nA;

  if noEvent(abs(cosBtwAxisAndVert) < cos_of_max and InPortB.r[2] < R) then
    OBPB = -(R1*sinq/cosq/cospsi)*ni - (R - R1/cosq)*nA;
    rB = InPortB.r + OBPB;
    j = 1; // IN CONTACT
  else
    OBPB = zeros(3); // does not matter
    if noEvent(cosBtwAxisAndVert > 0) then
      rB = InPortB.r - L2*ni;
      j = 2; // ??? roller is to the right of vertical
    else
      rB = InPortB.r + L2*ni;
      j = 3; // ??? roller is to the left of vertical
    end if;
  end if;

  rA = {rB[1], 0, rB[3]};
  h = rB[2];

  annotation (experiment(
      StopTime=10,
      NumberOfIntervals=50000,
      Tolerance=1e-009,
      Algorithm="Dassl"), experimentSetupOutput);
end RollerContactTrackingGeneralStep;
