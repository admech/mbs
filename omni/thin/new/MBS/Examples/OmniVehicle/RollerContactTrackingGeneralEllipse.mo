within MBS.Examples.OmniVehicle;

partial model RollerContactTrackingGeneralEllipse
  extends Constraint;
  parameter Integer n = 4 "Number of rollers";
  parameter Real R = 1 "Omni wheel outer radius (ellipse small axis)";
  parameter Real psi = 0 "Angle of roller distortion (fixed axis turn)";

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
  parameter Real cos_of_max = 0.65128; // For psi = 0.4
//  parameter Real cos_of_max = 0.6512884; // For psi = 0.4
//  parameter Real cos_of_max = n_at_max * {1, 0};
  SI.Position[3] rA; // A - plane
  SI.Position[3] rB; // B - roller

  Real[3] r_center "Ellipse center in global coords";

  // Intermediate coordinates:
  Real[3] ni "Roller axis in global coords";
  Real[3] crs "horizontal ni cross vertical";
  Real[3] nj "In vertical plane";
  Real[3] nk "Horizontal (along crs (singularity!))";
  Real[3,3] Ti "Intermediate to global transition";
  Real[3] rB_int
    "Closest point in intermed coord (from center to contact)";
  Real[3] nA_int "Vertical unit vector in intermediate coordinates";

//  Real[3] b;
  Real h;     // point of contact vertical coordinate
  Real j;
  Real cosBtwAxisAndVert;
  Real w;
equation
  w = abs(cosBtwAxisAndVert);

    // Intermediate coordinates:
  ni = InPortB.T*i;
  crs = cross(ni, nA);
  // nj singularity at || nA !
  Ti = [ni[1], nj[1], nk[1];
        ni[2], nj[2], nk[2];
        ni[3], nj[3], nk[3]];
  nA_int = transpose(Ti)*nA; // note that stuff doesn't work when not in contact

//  b = cross(InPortB.T*i, i);
  cosBtwAxisAndVert = ni*nA;

  if noEvent(abs(cosBtwAxisAndVert) < cos_of_max) then
    nk = crs/sqrt(crs*crs);
    nj = cross(nk, ni);
    r_center = InPortB.r + R1*nj;
    rB_int = - {nA_int[1]*Q^2, nA_int[2]*R^2, 0} / sqrt(nA_int[1]^2*Q^2 + nA_int[2]^2*R^2);
    rB = r_center + Ti*rB_int;
    j = 1; // IN CONTACT
  else
    nk = crs; // does not matter
    nj = cross(nk, ni);
    r_center = InPortB.r + R1*nj;
    if noEvent(cosBtwAxisAndVert > 0) then
      rB_int = {-L2, -R1, 0};
      rB = InPortB.r - L2*ni;
      j = 2; // ??? roller is to the right of vertical
    else
      rB_int = {L2, -R1, 0};
      rB = InPortB.r + L2*ni;
      j = 3; // ??? roller is to the left of vertical
    end if;
  end if;

  rA = {rB[1], 0, rB[3]};
  h = rB[2];
end RollerContactTrackingGeneralEllipse;
