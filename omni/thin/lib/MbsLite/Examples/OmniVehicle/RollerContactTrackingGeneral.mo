within MbsLite.Examples.OmniVehicle;

partial model RollerContactTrackingGeneral
  extends Constraint;

  parameter Integer  n = 4    "Number of rollers";
  parameter Real     R = 1    "Omni wheel outer radius (ellipse small axis)";
  parameter Real     psi = 1   "Angle of roller distortion (fixed axis turn)";

  parameter Real     alpha = Modelica.Constants.pi / n       "Max angle of the half-sector";
  parameter Real     Q = R / cos(psi)                        "Ellipse large axis";
  parameter Real     R1 = R * cos(alpha)                     "Omni wheel inner radius";
  parameter Real     L1 = R * sin(alpha)                     "Half roller visible (projection) length";
  parameter Real     L2 = L1 / cos(psi)                      "Half roller length";
  parameter Real[3]  nA = { 0, 1, 0 }                        "Roller over horizontal surface: vertical unit vector (in global coordinates)";
  parameter Real[2]  gradAtRollerTip = 2 * { L2 / Q^2, R1 / R^2 };
  parameter Real[2]  n_at_max = gradAtRollerTip / sqrt(gradAtRollerTip * gradAtRollerTip)   "Normal to ellipse in local coord (it is in vert plane)";
  parameter Real[3]  i = { 1, 0, 0 }                         "Roller axis of symmetry unit vector";
  parameter Real     cos_of_max = 0.3820515; // For psi = 1.0
  // parameter Real     cos_of_max = 0.44; // For psi = 0.9
  // parameter Real     cos_of_max = 0.4927; // For psi = 0.8
  // parameter Real     cos_of_max = 0.5408; // For psi = 0.7
  // parameter Real     cos_of_max = 0.584; // For psi = 0.6
  // parameter Real     cos_of_max = 0.62; // For psi = 0.5
  // parameter Real     cos_of_max = 0.65128; // For psi = 0.4
  // parameter Real     cos_of_max = n_at_max * {1, 0};
  // parameter Real     cos_of_max = cos(Modelica.Constants.pi/2 - alpha); // cos(pi/2 - pi/n)
  SI.Position[3]     rA; // A - plane
  SI.Position[3]     rB; // B - roller

  // Auxiliary roller base:
  Real[3]            ni      "Roller axis in global coords";
  Real[3]            crs     "horizontal ni cross vertical";
  Real[3]            nk      "Horizontal (along crs)";

  // Auxiliary wheel base
  Real[3]            n1j     "In vertical direction (= nA)";
  Real[3]            n1k     "Horizontal wheel axis, delivered from above";
  Real               lambda  "Parameter to be computed";
  Real[3]            rho     "Unit vector between mass centers";
  Real[3]            OBPB;

  Real               j;
  Real               cosBtwAxisAndVert;

equation

  // Intermediate wheel coordinates:
  n1j = nA;

  // Intermediate roller coordinates:
  ni = InPortB.T * i;
  crs = cross(ni, nA);
  nk = crs / sqrt(crs * crs);
  cosBtwAxisAndVert = ni * nA;

  if noEvent(abs(cosBtwAxisAndVert) < cos_of_max and InPortB.r[2] < R) then
    lambda = (R * (n1j * nk) - R1 * (rho * nk)) / (n1k * nk);
    OBPB = -R * n1j + lambda * n1k + R1 * rho;
    rB = InPortB.r + OBPB;
    j = 1; // IN CONTACT
  else
    lambda = R * (n1j * nk) / (n1k * nk);
    OBPB = -R * n1j;
    if noEvent(cosBtwAxisAndVert > 0) then
      rB = InPortB.r - L2 * ni;
      j = 2; // ??? roller is to the right of vertical
    else
      rB = InPortB.r + L2 * ni;
      j = 3; // ??? roller is to the left of vertical
    end if;
  end if;

  rA = { rB[1], 0, rB[3] };

  annotation
    ( experiment
        ( StopTime=10
        , NumberOfIntervals=50000
        , Tolerance=1e-009
        , Algorithm="Dassl"
        )
    , experimentSetupOutput
    );

end RollerContactTrackingGeneral;
