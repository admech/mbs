within MBS.Examples.OmniVehicle;

package Core

     model RollerOnPlaneSlipping
       extends SlidingWithDryFriction;
       parameter Real R = 1;
       parameter Real R1 = R/sqrt(2);
       Real xi;
     equation
       xi^2 = rhoB[2]^2 + rhoB[3]^2;
       (xi + R1)^2 + rhoB[1]^2 = R^2;
       rhoA[2] = 0;
       gradfA = {0, 1, 0};
       gradfB = 2*{rhoB[1], rhoB[2]*(xi + R1)/xi, rhoB[3]*(xi + R1)/xi};
     end RollerOnPlaneSlipping;
     
     model SlippingRollerTest
       parameter Real R = 1;
       parameter Real R1 = R/sqrt(2);
       parameter Real omega0 = 10;
       Base base annotation (Placement(transformation(extent={{-80,-20},{-40,20}})));
       OnePortHeavyBody onePortHeavyBody(
         Gravity = {0, -1, 0},
         r(start = {0, R - R1, 0}),
         v(start = {0, 0, 0}),
         q(start = {1, 0, 0, 0}),
         omega(start = {omega0, 0, 0})) 
         annotation (Placement(transformation(extent={{40,-20},{80,20}})));
       RollerOnPlaneSlipping rollerOnPlaneSlipping(
         xi(start = R - R1),
         rhoA(start = {0, 0, 0}),
         rhoB(start = {0, -(R - R1), 0}),
         lambda(start = -2*R)) 
         annotation (Placement(transformation(extent={{-20,-20},{20,20}})));
       annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                 -100},{100,100}}),
                              graphics),
         experiment(
           StopTime=10,
           NumberOfIntervals=50000,
           Tolerance=1e-006),
         experimentSetupOutput);
     equation
       connect(rollerOnPlaneSlipping.OutPortB, onePortHeavyBody.InPort) annotation (
           Line(
           points={{8,16},{8,40},{60,40},{60,16}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(rollerOnPlaneSlipping.InPortB, onePortHeavyBody.OutPort) annotation (
           Line(
           points={{8,-16},{8,-40},{60,-40},{60,-16}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(rollerOnPlaneSlipping.InPortA, base.OutPort) annotation (Line(
           points={{-8,-16},{-8,-40},{-60,-40},{-60,-16}},
           color={0,0,255},
           smooth=Smooth.None));
     end SlippingRollerTest;
     
     model RollerOverHorizontalSurface
       extends Constraint;
       parameter Real alpha = 1;
       parameter Real R = 1;
       parameter Real R1 = R*cos(alpha);
       parameter Real delta = 10^(-6);
       parameter Real fric = 0.1;
       parameter Real stiff = 100;
       parameter Real damp = 10;
       Real kappa;
       Real[3] vA;
       Real[3] vB;
       Real[3] vr;
       Real[3] vrtan;
       Real vrt;
       Real[3] RBt;
       Real RBn;
       SI.Position[3] rA;
       SI.Position[3] rB;
       Real h;
       Real[3] nA;
     equation
       h = 0;//
       rA = InPortA.r + InPortA.T*rhoA;
       rB = InPortB.r + InPortB.T*rhoB;
       nA = {0, 1, 0};
       vA = InPortA.v + cross(InPortA.omega, rA - InPortA.r);
       vB = InPortB.v + cross(InPortB.omega, rB - InPortB.r);
       vr = vB - vA;
       vrn = vr*nA;
       vrtan = vr - vrn*nA;
       vrt = sqrt(vrtan*vrtan);
       RBn = OutPortB.F*nA;
       RBn = if h >= 0 then 0 else -stiff*h*sqrt(abs(h)) - damp*vrn;
       RBt = -fric*vrtan*(if vrt <= delta then 1/delta else 1/vrt)*RBn + kappa*nA;
       OutPortB.F = RBt + RBn*nA;
       OutPortA.P = rA;
       OutPortB.P = rB;
       OutPortB.M = zeros(3);
     end RollerOverHorizontalSurface;
     
     partial model RollerContactTracking
       extends Constraint;
       parameter Integer n = 4 "Number of rollers";
       parameter Real alpha = Modelica.Constants.pi/n
         "Max angle of the half-sector";
       parameter Real cos_of_max = cos(Modelica.Constants.pi/2 - alpha); // cos(pi/2 - pi/n)
       parameter Real R = 1 "Omni wheel outer radius";
       parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
       parameter Real L1 = R*sin(alpha) "Half roller length";
       parameter Real[3] nA = {0, 1, 0}
         "Roller over horizontal surface: vertical unit vector";
       parameter Real[3] i = {1, 0, 0} "Roller axis of symmetry unit vector";
       parameter Real psi = 0 "Angle of roller distortion (fixed axis turn)";
       SI.Position[3] rA;
       SI.Position[3] rB;
     //  Real[3] b;
       Real[3] c;
       Real[3] d;
       Real h;
       Real j;
       Real cosBtwAxisAndVert;
     equation
     //  b = cross(InPortB.T*i, i);
       d = cross(InPortB.T*i, nA);
       cosBtwAxisAndVert = (InPortB.T*i)*nA;
     
       if noEvent(abs(cosBtwAxisAndVert) < cos_of_max) then
          c = R1*cross(d, InPortB.T*i)/sqrt(d*d);
          rB = InPortB.r + c - R*nA;
          j = 1; // IN CONTACT
       else
          if noEvent(cosBtwAxisAndVert > 0) then
            c = -L1*InPortB.T*i;
            j = 2; // ??? roller is to the right of vertical
          else
            c = L1*InPortB.T*i;
            j = 3; // ??? roller is to the left of vertical
          end if;
          rB = InPortB.r + c;
       end if;
     
       rA = {rB[1], 0, rB[3]};
       h = rB[2];
     end RollerContactTracking;
     
     partial model RollerContactTrackingGeneral
       extends Constraint;
       parameter Integer n = 4 "Number of rollers";
       parameter Real R = 1 "Omni wheel outer radius (ellipse small axis)";
       parameter Real psi = 1 "Angle of roller distortion (fixed axis turn)";
     
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
     
     // Auxiliary roller base:
       Real[3] ni "Roller axis in global coords";
       Real[3] crs "horizontal ni cross vertical";
       Real[3] nk "Horizontal (along crs)";
     
     // Auxiliary wheel base
       Real[3] n1j "In vertical direction (= nA)";
       Real[3] n1k "Horizontal wheel axis, delivered from above";
       Real lambda "Parameter to be computed";
       Real[3] rho "Unit vector between mass centers";
       Real[3] OBPB;
     
       Real j;
       Real cosBtwAxisAndVert;
     equation
     // Intermediate wheel coordinates:
       n1j = nA;
     
     // Intermediate roller coordinates:
       ni = InPortB.T*i;
       crs = cross(ni, nA);
       nk = crs/sqrt(crs*crs);
       cosBtwAxisAndVert = ni*nA;
     
       if noEvent(abs(cosBtwAxisAndVert) < cos_of_max and InPortB.r[2] < R) then
         lambda = (R*(n1j*nk) - R1*(rho*nk))/(n1k*nk);
         OBPB = -R*n1j + lambda*n1k + R1*rho;
         rB = InPortB.r + OBPB;
         j = 1; // IN CONTACT
       else
         lambda = R*(n1j*nk)/(n1k*nk);
         OBPB = -R*n1j;
         if noEvent(cosBtwAxisAndVert > 0) then
           rB = InPortB.r - L2*ni;
           j = 2; // ??? roller is to the right of vertical
         else
           rB = InPortB.r + L2*ni;
           j = 3; // ??? roller is to the left of vertical
         end if;
       end if;
     
       rA = {rB[1], 0, rB[3]};
     
       annotation (experiment(
           StopTime=10,
           NumberOfIntervals=50000,
           Tolerance=1e-009,
           Algorithm="Dassl"), experimentSetupOutput);
     end RollerContactTrackingGeneral;
     
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
     
     model OmniWheel
       parameter Integer n = 4 "Number of rollers";
       parameter Real alpha = Modelica.Constants.pi/n
         "Max angle of the half-sector";
       parameter Real R = 1 "Omni wheel outer radius";
       parameter Real R1 = R*cos(alpha) "Omni wheel inner radius";
       parameter Real[3] v0 = {1, 0, 0};
       parameter Real[3] omega0 = {0, 0, -1};
       parameter Real pi = Modelica.Constants.pi;
       TwoPortsHeavyBody Roller0(
         Gravity = {0, -1, 0},
         r(start = {0, R - R1,0}),
         v(start = v0 + cross(omega0, {0, -R1, 0})),
         q(start = {1, 0, 0, 0}),
         omega(start = omega0)) 
         annotation (Placement(transformation(extent={{-30,-70},{-10,-50}})));
       RollerContactForces Contact0 
         annotation (Placement(transformation(extent={{-60,-70},{-40,-50}})));
       annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                 -100},{100,100}}),
                              graphics),
         experiment(
           StopTime=40,
           NumberOfIntervals=50000,
           Tolerance=1e-008),
         experimentSetupOutput);
       RollerContactForces Contact1 
         annotation (Placement(transformation(extent={{-60,-30},{-40,-10}})));
       TwoPortsHeavyBody Roller1(
         Gravity = {0, -1, 0},
         r(start = {R1, R, 0}),
         v(start = v0 + cross(omega0, {R1, 0, 0})),
         q(start = {cos(pi/4), 0, 0, sin(pi/4)}),
         omega(start = omega0)) 
         annotation (Placement(transformation(extent={{-30,-30},{-10,-10}})));
       RollerContactForces Contact2 
         annotation (Placement(transformation(extent={{-60,10},{-40,30}})));
       TwoPortsHeavyBody Roller2(
         Gravity = {0, -1, 0},
         r(start = {0, R + R1, 0}),
         v(start = v0 + cross(omega0, {0, R1, 0})),
         q(start = {cos(pi/2), 0, 0, sin(pi/2)}),
         omega(start = omega0)) 
         annotation (Placement(transformation(extent={{-30,10},{-10,30}})));
       RollerContactForces Contact3 
         annotation (Placement(transformation(extent={{-60,50},{-40,70}})));
       TwoPortsHeavyBody Roller3(
         Gravity = {0, -1, 0},
         r(start = {-R1, R, 0}),
         v(start = v0 + cross(omega0, {-R1, 0, 0})),
         q(start = {cos(3*pi/4), 0, 0, sin(3*pi/4)}),
         omega(start = omega0)) 
         annotation (Placement(transformation(extent={{-30,50},{-10,70}})));
       FixedJoint Joint3(
         nA = {1, 0, 0},
         nB = {0, -1, 0},
         rA = {0, 0, 0},
         rB = {-R1, 0, 0}) 
         annotation (Placement(transformation(extent={{0,50},{20,70}})));
       FixedJoint Joint2(
         nA = {1, 0, 0},
         nB = {-1, 0, 0},
         rA = {0, 0, 0},
         rB = {0, R1, 0}) 
         annotation (Placement(transformation(extent={{0,10},{20,30}})));
       FixedJoint Joint1(
         nA = {1, 0, 0},
         nB = {0, 1, 0},
         rA = {0, 0, 0},
         rB = {R1, 0, 0}) 
         annotation (Placement(transformation(extent={{0,-30},{20,-10}})));
       FixedJoint Joint0(
         nA = {1, 0, 0},
         nB = {1, 0, 0},
         rA = {0, 0, 0},
         rB = {0, -R1, 0}) 
         annotation (Placement(transformation(extent={{0,-70},{20,-50}})));
       FivePortsHeavyBody Wheel(
         Gravity = {0, -1, 0},
         r(start = {0, R, 0}),
         v(start = v0),
         q(start = {1, 0, 0, 0}),
         omega(start = omega0)) 
         annotation (Placement(transformation(extent={{40,-10},{60,10}})));
       KinematicPort InPortK 
         annotation (Placement(transformation(extent={{-50,-100},{-30,-80}})));
       WrenchPort InPortF 
         annotation (Placement(transformation(extent={{30,80},{50,100}})));
       KinematicPort OutPortK 
         annotation (Placement(transformation(extent={{30,-100},{50,-80}})));
     equation
       connect(Contact0.InPortB, Roller0.OutPort) 
         annotation (Line(
           points={{-46,-68},{-46,-76},{-20,-76},{-20,-68}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Contact0.OutPortB, Roller0.InPort) 
         annotation (Line(
           points={{-46,-52},{-46,-44},{-24,-44},{-24,-52.1}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Contact1.InPortB, Roller1.OutPort) annotation (Line(
           points={{-46,-28},{-46,-36},{-20,-36},{-20,-28}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Contact1.OutPortB, Roller1.InPort) annotation (Line(
           points={{-46,-12},{-46,-4},{-24,-4},{-24,-12.1}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Contact2.InPortB, Roller2.OutPort) annotation (Line(
           points={{-46,12},{-46,4},{-20,4},{-20,12}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Contact2.OutPortB, Roller2.InPort) annotation (Line(
           points={{-46,28},{-46,36},{-24,36},{-24,27.9}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Contact3.InPortB, Roller3.OutPort) annotation (Line(
           points={{-46,52},{-46,44},{-20,44},{-20,52}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Contact3.OutPortB, Roller3.InPort) annotation (Line(
           points={{-46,68},{-46,76},{-24,76},{-24,67.9}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Roller3.InPort1, Joint3.OutPortA) annotation (Line(
           points={{-16,68},{-16,76},{6,76},{6,68}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Roller2.InPort1, Joint2.OutPortA) annotation (Line(
           points={{-16,28},{-16,36},{6,36},{6,28}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Roller1.InPort1, Joint1.OutPortA) annotation (Line(
           points={{-16,-12},{-16,-4},{6,-4},{6,-12}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Roller2.OutPort, Joint2.InPortA) annotation (Line(
           points={{-20,12},{-20,4},{6,4},{6,12}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Roller3.OutPort, Joint3.InPortA) annotation (Line(
           points={{-20,52},{-20,44},{6,44},{6,52}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Roller0.InPort1, Joint0.OutPortA) annotation (Line(
           points={{-16,-52},{-16,-44},{6,-44},{6,-52}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Roller0.OutPort, Joint0.InPortA) annotation (Line(
           points={{-20,-68},{-20,-76},{6,-76},{6,-68}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Roller1.OutPort, Joint1.InPortA) annotation (Line(
           points={{-20,-28},{-20,-36},{6,-36},{6,-28}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Wheel.InPort, Joint0.OutPortB) annotation (Line(
           points={{43.4,8},{44,8},{44,14},{36,14},{36,-44},{14,-44},{14,-52}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Wheel.InPort2, Joint2.OutPortB) annotation (Line(
           points={{52.2,8},{52.2,36},{14,36},{14,28}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Wheel.InPort1, Joint1.OutPortB) annotation (Line(
           points={{47.8,8},{48,8},{48,18},{32,18},{32,-4},{14,-4},{14,-12}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Wheel.InPort3, Joint3.OutPortB) annotation (Line(
           points={{56.6,8},{56,8},{56,76},{14,76},{14,68}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Wheel.OutPort, Joint3.InPortB) annotation (Line(
           points={{50,-8},{50,-16},{28,-16},{28,44},{14,44},{14,52}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Wheel.OutPort, Joint2.InPortB) annotation (Line(
           points={{50,-8},{50,-16},{28,-16},{28,4},{14,4},{14,12}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Wheel.OutPort, Joint1.InPortB) annotation (Line(
           points={{50,-8},{50,-36},{14,-36},{14,-28}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Wheel.OutPort, Joint0.InPortB) annotation (Line(
           points={{50,-8},{50,-76},{14,-76},{14,-68}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Wheel.InPort4, InPortF) annotation (Line(
           points={{58,0},{70,0},{70,90},{40,90}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Wheel.OutPort, OutPortK) annotation (Line(
           points={{50,-8},{50,-76},{40,-76},{40,-90}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Contact1.InPortA, InPortK) annotation (Line(
           points={{-54,-28},{-54,-36},{-70,-36},{-70,-90},{-40,-90}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Contact0.InPortA, InPortK) annotation (Line(
           points={{-54,-68},{-56,-68},{-56,-90},{-40,-90}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Contact2.InPortA, InPortK) annotation (Line(
           points={{-54,12},{-54,4},{-70,4},{-70,-90},{-40,-90}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Contact3.InPortA, InPortK) annotation (Line(
           points={{-54,52},{-54,44},{-70,44},{-70,-90},{-40,-90}},
           color={0,0,255},
           smooth=Smooth.None));
     end OmniWheel;
     
     model ThreeWheelsVehicle
       OmniWheel Wheel1 
         annotation (Placement(transformation(extent={{-46,40},{-6,80}})));
       OmniWheel Wheel2 
         annotation (Placement(transformation(extent={{-46,-20},{-6,20}})));
       OmniWheel Wheel3 
         annotation (Placement(transformation(extent={{-46,-80},{-6,-40}})));
       Base Floor 
                 annotation (Placement(transformation(extent={{-96,-20},{-56,
                 20}})));
       FixedJoint Joint1(
         nA = {1, 0, 0},
         nB = {0, 1, 0},
         rA = {0, 0, 0},
         rB = {R1, 0, 0}) 
         annotation (Placement(transformation(extent={{4,40},{44,80}})));
       FixedJoint Joint2(
         nA = {1, 0, 0},
         nB = {0, 1, 0},
         rA = {0, 0, 0},
         rB = {R1, 0, 0}) 
         annotation (Placement(transformation(extent={{4,-20},{44,20}})));
       FixedJoint Joint3(
         nA = {1, 0, 0},
         nB = {0, 1, 0},
         rA = {0, 0, 0},
         rB = {R1, 0, 0}) 
         annotation (Placement(transformation(extent={{4,-80},{44,-40}})));
       ThreePortsHeavyBody Body 
         annotation (Placement(transformation(extent={{54,-20},{94,20}})));
       annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                 -100},{100,100}}),       graphics));
     equation
       connect(Wheel1.InPortK, Floor.OutPort) annotation (Line(
           points={{-34,42},{-34,34},{-52,34},{-52,-26},{-76,-26},{-76,-16}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Wheel2.InPortK, Floor.OutPort) annotation (Line(
           points={{-34,-18},{-34,-26},{-76,-26},{-76,-16}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Wheel3.InPortK, Floor.OutPort) annotation (Line(
           points={{-34,-78},{-34,-86},{-52,-86},{-52,-26},{-76,-26},{-76,-16}},
           color={0,0,255},
           smooth=Smooth.None));
     
       connect(Wheel1.InPortF, Joint1.OutPortA) annotation (Line(
           points={{-18,78},{-18,86},{16,86},{16,76}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Wheel2.InPortF, Joint2.OutPortA) annotation (Line(
           points={{-18,18},{-18,26},{16,26},{16,16}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Wheel3.InPortF, Joint3.OutPortA) annotation (Line(
           points={{-18,-42},{-18,-34},{16,-34},{16,-44}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Wheel3.OutPortK, Joint3.InPortA) annotation (Line(
           points={{-18,-78},{-18,-86},{16,-86},{16,-76}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Wheel2.OutPortK, Joint2.InPortA) annotation (Line(
           points={{-18,-18},{-18,-26},{16,-26},{16,-16}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Wheel1.OutPortK, Joint1.InPortA) annotation (Line(
           points={{-18,42},{-18,34},{16,34},{16,44}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Joint2.OutPortB, Body.InPort) annotation (Line(
           points={{32,16},{32,26},{62,26},{62,16}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Joint1.OutPortB, Body.InPort1) annotation (Line(
           points={{32,76},{32,86},{74,86},{74,16}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Joint3.OutPortB, Body.InPort2) annotation (Line(
           points={{32,-44},{32,-34},{96,-34},{96,26},{86,26},{86,16}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Body.OutPort, Joint2.InPortB) annotation (Line(
           points={{74,-16},{74,-26},{32,-26},{32,-16}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Body.OutPort, Joint1.InPortB) annotation (Line(
           points={{74,-16},{74,-26},{48,-26},{48,34},{32,34},{32,44}},
           color={0,0,255},
           smooth=Smooth.None));
       connect(Body.OutPort, Joint3.InPortB) annotation (Line(
           points={{74,-16},{74,-26},{48,-26},{48,-86},{32,-86},{32,-76}},
           color={0,0,255},
           smooth=Smooth.None));
     end ThreeWheelsVehicle;
     
     model NonHolonomicOmniPlatform
       "Modelica implementation of non-holonomic model by Borisov."
       import Modelica.Constants.pi;
      /*
     Moving axes Oe_1e_2, fixed in the platform.
     */
     
      // INITIAL VALUES -------------------------------------------------------------
      parameter Real[2] __0_v =  {1, 0};
      parameter Real __0_omega =   0;
      parameter Real __0_M =    0;
      // ----------------------------------------------------------------------------
     
      // ----------------------------------------------------------------------------
      parameter Real[2,2] E =   identity(2) "Identity matrix.";
      parameter Real[2,2] J =   [0, -1; 1, 0]
         "Skew matrix for vector crossing.";
      // ----------------------------------------------------------------------------
     
      // GEOMETRY -------------------------------------------------------------------
      parameter Real psi = 0 "Angle btw roller axis and wheel plane";
      parameter Integer N_WHEELS =   3 "Number of wheels.";
      parameter Real[N_WHEELS] wheelAngles = {2*pi/N_WHEELS*(i-1) for i in 1:N_WHEELS}
         "Directions to wheels from Platform center.";
      parameter Real platformDiameter =    1;
      parameter Real[2] rc =   {0, 0}
         "Masscenter radius-vector. RelLoc (Relative wrt moving).";
      parameter Real[N_WHEELS,2] r = {platformDiameter * {cos(wheelAngles[i]), sin(wheelAngles[i])} for i in 1:N_WHEELS}
         "Wheel radius-vectors. RelLoc.";
      parameter Real[N_WHEELS] delta = {pi/2 - psi for i in 1:N_WHEELS}
         "Pi/2 minus psi (psi is angle btw roller axis and wheel plane).";
      parameter Real[N_WHEELS,2] alpha = {{cos(delta[i]+wheelAngles[i]), sin(delta[i]+wheelAngles[i])} for i in 1:N_WHEELS}
         "Roller axes. RelLoc?";
      parameter Real[N_WHEELS] s = {sin(delta[i]) for i in 1:N_WHEELS}
         "sin(delta) (delta = pi/2 - ang btw axis and wheel).";
      parameter Real[N_WHEELS] h = {1 for i in 1:N_WHEELS} "Wheel radius.";
     
      Real[N_WHEELS, 2,2] alphaTensor = {{alpha[k,i]*alpha[k,j] for i in 1:2, j in 1:2} for k in 1:N_WHEELS}
         "alpha[i] (x) alpha[i] matrices (aT[k,i,j] = alpha[k,i]*alpha[k,j]).";
      // ----------------------------------------------------------------------------
     
      // INERTIA --------------------------------------------------------------------
      parameter Real[N_WHEELS] mi = {1 for i in 1:N_WHEELS} "Wheel masses.";
      parameter Real m0 =    10 "Platform mass.";
      parameter Real m =    m0 + sum(mi[i] for i in 1:N_WHEELS)
         "Total mass of the system.";
     
      parameter Real Ikr =    I + sum(Ii[i]/((s[i]^2)*h[i])* (J*r[i,:] * alpha[i,:])*(J*r[i,:] * alpha[i,:]) for i in 1:N_WHEELS)
         "Inertia, see other declarations. (I s kryshechkoj).";
      parameter Real I =    I0 + sum((mi[i]*r[i,:]*r[i,:] + Iiw[i]) for i in 1:N_WHEELS)
         "Total system inertia wrt point O.";
      parameter Real[N_WHEELS] Ii =  {1 for i in 1:N_WHEELS}
         "Wheel inertia wrt wheel axis. (I_i)";
      parameter Real[N_WHEELS] Iiw = {1/2 for i in 1:N_WHEELS}
         "Wheel inertia wrt wheel diameter (I with wave - s volnoj).";
      parameter Real I0 =    10 "Platform inertia.";
      // ----------------------------------------------------------------------------
     
      Real[2,2] Gamma
         "Roller axes' tensor multiplication matrix (see equations)";
      Real[2] v(   start = __0_v)
         "Velocity of point O, origin of moving axes. AbsLoc (Absolute wrt moving frame).";
      Real[2] a "Acceleration of point O, origin of moving axes. AbsLoc.";
      Real phi "Platform turn angle, grows counter-clockwise.";
      Real omega(   start = __0_omega)
         "Platform turn velocity, positive if counter-clockwise.";
      Real epsilon
         "Platform turn acceleration, positive if counter-clockwise.";
      Real[2] R "Hell of a rotational thing. See equations.";
      Real[N_WHEELS] mu "Normalized torques applied to wheel axes.";
      Real x "Masscenter X AbsGlo ?";
      Real y "Masscenter Y AbsGlo ?";
      Real[N_WHEELS] M "Torques applied to wheel axes.";
     
      Real[N_WHEELS] wheel_angle "Wheel turn angles";
     
     equation
      // MOTION EQUATIONS
      (Gamma + m*E)*a + m*epsilon*(J*rc + R) + m*omega*J*(v + omega*J*rc)
       = -sum(mu[i]*alpha[i,:] for i in 1:N_WHEELS);
     
      Ikr*epsilon + m*(J*rc + R)*a + m*omega*v*rc = -sum(mu[i]*((J*r[i,:])*alpha[i,:]) for i in 1:N_WHEELS);
     
      der(x) = v[1]*cos(phi) - v[2]*sin(phi);
      der(y) = v[1]*sin(phi) + v[2]*cos(phi);
     
      der(phi) = omega;
     
      der(v) = a;
      der(omega) = epsilon;
      // der(rc) = v; //?
     
      // HELPER
      Gamma = sum(Ii[i]/((s[i]*h[i])^2)*alphaTensor[i,:,:] for i in 1:N_WHEELS);
      R = 1/m*sum(Ii[i]/((s[i]*h[i])^2) * (J*r[i,:] * alpha[i,:])*alpha[i,:] for i in 1:N_WHEELS);
      mu = {M[i]/(s[i]*h[i]) for i in 1:N_WHEELS};
     
      for i in 1:N_WHEELS loop
        der(wheel_angle[i]) = -1/(s[i]*h[i]) * (v + omega*J*r[i,:])*alpha[i,:];
      end for;
     
      // PARAMETERS
      M = {__0_M for i in 1:N_WHEELS};
     
     end NonHolonomicOmniPlatform;
     
     model Comparison
       "Compares OmniWheelSet physical MBS model with Borisov non-holonomic."
       import Modelica.Constants.pi;
     
       parameter Real __0_v_x =             0;
       parameter Real __0_v_y =             0;
       parameter Real __0_omega =           1;
       parameter Integer __n_rollers =      4;
       parameter Real __rollerFraction =    1e-1;
     
       parameter Real platformDiameter = 1;
       parameter Real wheelMass =        1;
       parameter Real platformMass =     3.3;
       parameter Real wheelRadius =      1;
       parameter Real wheelAspectRatio = 1/5 "Thickness/Radius";
       parameter Real[3] wheelInertia =  wheelMass * (wheelRadius^2) * {1/2, 1/4 + 1/12*wheelAspectRatio^2, 1/4 + 1/12*wheelAspectRatio^2};
       //MBS prokidyvaet whIn stupize, a rolleram delaet individ.
       parameter Real platformInertia =  platformMass * ((platformDiameter/2)^2) / 2;
     
     //  parameter Real[3] rollerInertia = rollerMass * {1/2*(R-R1)^2, 3/12*(R-R1)^2 + 1/12*(2*R*sin(alpha))^2, 3/12*(R-R1)^2 + 1/12*(2*R*sin(alpha))^2};
     
       //should duplicate same from Wheel class:
       parameter Real rollerMass = wheelMass*__rollerFraction/__n_rollers;
     //  parameter Real alpha = pi/__n_rollers "Max angle of the half-sector";
     //  parameter Real[__n_rollers,4] roller_q0s_local = {{cos(-alpha*(i-1)), 0, 0, -1 * sin(-alpha*(i-1))} for i in 1:__n_rollers};
     //  parameter Real[__n_rollers,4] roller_q0s =       {QMult1({1,0,0,0}, roller_q0s_local[i,:]) for i in 1:__n_rollers};
       parameter Real[3] rollerInertia = rollerMass * {1/2, 1, 1};
     //  parameter Real R1 = wheelRadius*cos(alpha) "Omni wheel inner radius";
     //  parameter Real[__n_rollers,3] rollerDirs = {{cos(-pi/2 + 2*alpha*(i-1)), sin(-pi/2 + 2*alpha*(i-1)), 0} for i in 1:__n_rollers};
     //  parameter Real[__n_rollers,3,3] roller_T0s = {QToT(roller_q0s[i,:]) for i in 1:__n_rollers};
     //  parameter Real[__n_rollers] x = {rollerDirs[i,1]*R1 for i in 1:__n_rollers}
     //    "Roller x_0";
     //  parameter Real[__n_rollers] y = {rollerDirs[i,2]*R1 for i in 1:__n_rollers}
     //    "Roller y_0";
     //  parameter Real[__n_rollers] z = {rollerDirs[i,3]*R1 for i in 1:__n_rollers}
     //    "Roller z_0";
     //  parameter Real[3,3] nhlTotalWheelInertia = diagonal(wheelInertia) + sum(( transpose(roller_T0s[i,:,:])*diagonal(rollerInertia)*roller_T0s[i,:,:] +
     //                                                                            rollerMass * [ x[i]^2,                     -(x[i]^2+y[i]^2+x[i]*y[i]), -(x[i]^2+z[i]^2+x[i]*z[i]);
     //                                                                                           -(x[i]^2+y[i]^2+x[i]*y[i]), y[i]^2,                     -(y[i]^2+z[i]^2+y[i]*z[i]);
     //                                                                                           -(x[i]^2+z[i]^2+x[i]*z[i]), -(y[i]^2+z[i]^2+y[i]*z[i]), z[i]^2])                        for i in 1:__n_rollers);
       parameter Real nhlTotalWheelAxisInertia = wheelInertia[1] + 3*((wheelRadius*cos(pi/__n_rollers))^2)*rollerInertia[1];
     
       Real nhl_x;
       Real nhl_y;
       Real nhl_angle;
       Real[3] nhl_wheelAngles;
     
       Real mbs_x;
       Real mbs_y;
       Real mbs_angle;
       Real[3] mbs_wheelAngles;
       Real[3] mbs_vt;
       Real mbs_platform_r2;
       Real[3] mbs_omega;
       Real[3] mbs_rollerInContactNumber;
       Real[3,__n_rollers] mbs_contacts_h;
       Real[3] mbs_forceTsqrt;
     
       Real delta_x;
       Real delta_y;
       Real delta_angle;
       Real[3] delta_wheelAngles;
     
       NonHolonomicOmniPlatform nhl(
          __0_v =  {__0_v_x, __0_v_y},
          __0_omega = __0_omega,
          m0 = platformMass,
          mi = {wheelMass for i in 1:3},
          Ii = {nhlTotalWheelAxisInertia for i in 1:3},
          Iiw = {wheelInertia[2] for i in 1:3},
          I0 = platformInertia);
     //     Ii = {wheelInertia[1] for i in 1:3},
     
       PointContact.NRollersPointContactOmniWheelSetTest mbs(
          v0 = {__0_v_x, 0, __0_v_y},
          om0 = __0_omega,
          rollerFraction = __rollerFraction,
          n = __n_rollers,
          platformDiameter = platformDiameter,
          wheelRadius = wheelRadius,
          wheelMass = wheelMass,
          platformMass = platformMass,
          wheelInertia = {wheelInertia[2], wheelInertia[2], wheelInertia[1]},
          platformInertia = platformInertia);
     
     equation
       nhl_x     = nhl.x;
       nhl_y     = nhl.y;
       nhl_angle = nhl.phi*180/pi;
       nhl_wheelAngles = nhl.psi*180/pi;
     
       mbs_x     = mbs._r1;
       mbs_y     = mbs._r3;
       mbs_angle = mbs._angle0;
       mbs_wheelAngles = {mbs._angle1, mbs._angle2, mbs._angle3};
       mbs_vt = {mbs.Wheel1._vt, mbs.Wheel2._vt, mbs.Wheel3._vt};
       mbs_platform_r2 = mbs.Platform.r[2];
       mbs_omega = mbs.Platform.omega;
       mbs_rollerInContactNumber = {mbs.Wheel1._rollerInContactNumber, mbs.Wheel2._rollerInContactNumber, mbs.Wheel3._rollerInContactNumber};
       mbs_contacts_h = {{mbs.Wheel1.Contacts[i].h for i in 1:__n_rollers},
                         {mbs.Wheel2.Contacts[i].h for i in 1:__n_rollers},
                         {mbs.Wheel3.Contacts[i].h for i in 1:__n_rollers}};
       mbs_forceTsqrt = {sum(mbs.Wheel1.Contacts[i].ForceTsqrt*mbs.Wheel1.Contacts[i].isInContact for i in 1:__n_rollers),
                         sum(mbs.Wheel2.Contacts[i].ForceTsqrt*mbs.Wheel2.Contacts[i].isInContact for i in 1:__n_rollers),
                         sum(mbs.Wheel3.Contacts[i].ForceTsqrt*mbs.Wheel3.Contacts[i].isInContact for i in 1:__n_rollers)};
     
       delta_x     = mbs_x - nhl_x;
       delta_y     = mbs_y - nhl_y;
       delta_angle = mbs_angle - nhl_angle;
       delta_wheelAngles = mbs_wheelAngles - nhl_wheelAngles;
     
       annotation (experiment(StopTime=10, Tolerance=1e-007),
           __Dymola_experimentSetupOutput);
     end Comparison;
     
     model ComparisonMbsAndMbsGeneral
       "Compares OmniWheelSet physical MBS model with Borisov non-holonomic."
       import Modelica.Constants.pi;
     
       parameter Real __0_v_x =             0;
       parameter Real __0_v_y =             0;
       parameter Real __0_omega =           1;
       parameter Integer __n_rollers =      4;
       parameter Real __rollerFraction =    1e-1;
     
       parameter Real platformDiameter = 1;
       parameter Real wheelMass =        1;
       parameter Real platformMass =     3.3;
       parameter Real wheelRadius =      1;
       parameter Real wheelAspectRatio = 1/5 "Thickness/Radius";
       parameter Real[3] wheelInertia =  wheelMass * (wheelRadius^2) * {1/2, 1/4 + 1/12*wheelAspectRatio^2, 1/4 + 1/12*wheelAspectRatio^2};
       //MBS prokidyvaet whIn stupize, a rolleram delaet individ.
       parameter Real platformInertia =  platformMass * ((platformDiameter/2)^2) / 2;
     
     //  parameter Real[3] rollerInertia = rollerMass * {1/2*(R-R1)^2, 3/12*(R-R1)^2 + 1/12*(2*R*sin(alpha))^2, 3/12*(R-R1)^2 + 1/12*(2*R*sin(alpha))^2};
     
       //should duplicate same from Wheel class:
       parameter Real rollerMass = wheelMass*__rollerFraction/__n_rollers;
     //  parameter Real alpha = pi/__n_rollers "Max angle of the half-sector";
     //  parameter Real[__n_rollers,4] roller_q0s_local = {{cos(-alpha*(i-1)), 0, 0, -1 * sin(-alpha*(i-1))} for i in 1:__n_rollers};
     //  parameter Real[__n_rollers,4] roller_q0s =       {QMult1({1,0,0,0}, roller_q0s_local[i,:]) for i in 1:__n_rollers};
       parameter Real[3] rollerInertia = rollerMass * {1/2, 1, 1};
     //  parameter Real R1 = wheelRadius*cos(alpha) "Omni wheel inner radius";
     //  parameter Real[__n_rollers,3] rollerDirs = {{cos(-pi/2 + 2*alpha*(i-1)), sin(-pi/2 + 2*alpha*(i-1)), 0} for i in 1:__n_rollers};
     //  parameter Real[__n_rollers,3,3] roller_T0s = {QToT(roller_q0s[i,:]) for i in 1:__n_rollers};
     //  parameter Real[__n_rollers] x = {rollerDirs[i,1]*R1 for i in 1:__n_rollers}
     //    "Roller x_0";
     //  parameter Real[__n_rollers] y = {rollerDirs[i,2]*R1 for i in 1:__n_rollers}
     //    "Roller y_0";
     //  parameter Real[__n_rollers] z = {rollerDirs[i,3]*R1 for i in 1:__n_rollers}
     //    "Roller z_0";
     //  parameter Real[3,3] nhlTotalWheelInertia = diagonal(wheelInertia) + sum(( transpose(roller_T0s[i,:,:])*diagonal(rollerInertia)*roller_T0s[i,:,:] +
     //                                                                            rollerMass * [ x[i]^2,                     -(x[i]^2+y[i]^2+x[i]*y[i]), -(x[i]^2+z[i]^2+x[i]*z[i]);
     //                                                                                           -(x[i]^2+y[i]^2+x[i]*y[i]), y[i]^2,                     -(y[i]^2+z[i]^2+y[i]*z[i]);
     //                                                                                           -(x[i]^2+z[i]^2+x[i]*z[i]), -(y[i]^2+z[i]^2+y[i]*z[i]), z[i]^2])                        for i in 1:__n_rollers);
       parameter Real nhlTotalWheelAxisInertia = wheelInertia[1] + 3*((wheelRadius*cos(pi/__n_rollers))^2)*rollerInertia[1];
     
       parameter Real _psi = pi/6;
     
       Real mbs_x;
       Real mbs_y;
       Real mbs_angle;
       Real[3] mbs_wheelAngles;
     
       Real mbsGeneral_x;
       Real mbsGeneral_y;
       Real mbsGeneral_angle;
       Real[3] mbsGeneral_wheelAngles;
     
       Real[3] mbs_vt;
       Real mbs_platform_r2;
       Real[3] mbs_omega;
       Real[3] mbs_rollerInContactNumber;
       Real[3,__n_rollers] mbs_contacts_h;
       Real[3] mbs_forceTsqrt;
     
       Real delta_x;
       Real delta_y;
       Real delta_angle;
       Real[3] delta_wheelAngles;
     
       PointContact.NRollersPointContactOmniWheelSetTest mbs(
          v0 = {__0_v_x, 0, __0_v_y},
          om0 = __0_omega,
          rollerFraction = __rollerFraction,
          n = __n_rollers,
          platformDiameter = platformDiameter,
          wheelRadius = wheelRadius,
          wheelMass = wheelMass,
          platformMass = platformMass,
          wheelInertia = {wheelInertia[2], wheelInertia[2], wheelInertia[1]},
          platformInertia = platformInertia);
     
       PointContact.NRollersPointContactOmniWheelSetTestGeneral mbsGeneral(
          v0 = {__0_v_x, 0, __0_v_y},
          om0 = __0_omega,
          rollerFraction = __rollerFraction,
          n = __n_rollers,
          psi = _psi,
          platformDiameter = platformDiameter,
          wheelRadius = wheelRadius,
          wheelMass = wheelMass,
          platformMass = platformMass,
          wheelInertia = {wheelInertia[2], wheelInertia[2], wheelInertia[1]},
          platformInertia = platformInertia);
     
     equation
       mbs_x     = mbs._r1;
       mbs_y     = mbs._r3;
       mbs_angle = mbs._angle0;
       mbs_wheelAngles = {mbs._angle1, mbs._angle2, mbs._angle3};
     
       mbsGeneral_x     = mbsGeneral._r1;
       mbsGeneral_y     = mbsGeneral._r3;
       mbsGeneral_angle = mbsGeneral._angle0;
       mbsGeneral_wheelAngles = {mbsGeneral._angle1, mbsGeneral._angle2, mbsGeneral._angle3};
     
       mbs_vt = {mbs.Wheel1._vt, mbs.Wheel2._vt, mbs.Wheel3._vt};
       mbs_platform_r2 = mbs.Platform.r[2];
       mbs_omega = mbs.Platform.omega;
       mbs_rollerInContactNumber = {mbs.Wheel1._rollerInContactNumber, mbs.Wheel2._rollerInContactNumber, mbs.Wheel3._rollerInContactNumber};
       mbs_contacts_h = {{mbs.Wheel1.Contacts[i].h for i in 1:__n_rollers},
                         {mbs.Wheel2.Contacts[i].h for i in 1:__n_rollers},
                         {mbs.Wheel3.Contacts[i].h for i in 1:__n_rollers}};
       mbs_forceTsqrt = {sum(mbs.Wheel1.Contacts[i].ForceTsqrt*mbs.Wheel1.Contacts[i].isInContact for i in 1:__n_rollers),
                         sum(mbs.Wheel2.Contacts[i].ForceTsqrt*mbs.Wheel2.Contacts[i].isInContact for i in 1:__n_rollers),
                         sum(mbs.Wheel3.Contacts[i].ForceTsqrt*mbs.Wheel3.Contacts[i].isInContact for i in 1:__n_rollers)};
     
       delta_x     = mbs_x - mbsGeneral_x;
       delta_y     = mbs_y - mbsGeneral_y;
       delta_angle = mbs_angle - mbsGeneral_angle;
       delta_wheelAngles = mbs_wheelAngles - mbsGeneral_wheelAngles;
     
       annotation (experiment(StopTime=10, Tolerance=1e-007),
           __Dymola_experimentSetupOutput);
     end ComparisonMbsAndMbsGeneral;
     
     model ComparisonBorisovAndMbsGeneral
       "Compares OmniWheelSet physical MBS model with Borisov non-holonomic."
       import Modelica.Constants.pi;
     
       parameter Real __0_v_x =             0;
       parameter Real __0_v_y =             0;
       parameter Real __0_omega =           1;
       parameter Integer __n_rollers =      4;
       parameter Real __rollerFraction =    1e-1;
     
       parameter Real platformDiameter = 1;
       parameter Real wheelMass =        1;
       parameter Real platformMass =     3.3;
       parameter Real wheelRadius =      1;
       parameter Real wheelAspectRatio = 1/5 "Thickness/Radius";
       parameter Real[3] wheelInertia =  wheelMass * (wheelRadius^2) * {1/2, 1/4 + 1/12*wheelAspectRatio^2, 1/4 + 1/12*wheelAspectRatio^2};
       //MBS prokidyvaet whIn stupize, a rolleram delaet individ.
       parameter Real platformInertia =  platformMass * ((platformDiameter/2)^2) / 2;
     
     //  parameter Real[3] rollerInertia = rollerMass * {1/2*(R-R1)^2, 3/12*(R-R1)^2 + 1/12*(2*R*sin(alpha))^2, 3/12*(R-R1)^2 + 1/12*(2*R*sin(alpha))^2};
     
       //should duplicate same from Wheel class:
       parameter Real rollerMass = wheelMass*__rollerFraction/__n_rollers;
     //  parameter Real alpha = pi/__n_rollers "Max angle of the half-sector";
     //  parameter Real[__n_rollers,4] roller_q0s_local = {{cos(-alpha*(i-1)), 0, 0, -1 * sin(-alpha*(i-1))} for i in 1:__n_rollers};
     //  parameter Real[__n_rollers,4] roller_q0s =       {QMult1({1,0,0,0}, roller_q0s_local[i,:]) for i in 1:__n_rollers};
       parameter Real[3] rollerInertia = rollerMass * {1/2, 1, 1};
     //  parameter Real R1 = wheelRadius*cos(alpha) "Omni wheel inner radius";
     //  parameter Real[__n_rollers,3] rollerDirs = {{cos(-pi/2 + 2*alpha*(i-1)), sin(-pi/2 + 2*alpha*(i-1)), 0} for i in 1:__n_rollers};
     //  parameter Real[__n_rollers,3,3] roller_T0s = {QToT(roller_q0s[i,:]) for i in 1:__n_rollers};
     //  parameter Real[__n_rollers] x = {rollerDirs[i,1]*R1 for i in 1:__n_rollers}
     //    "Roller x_0";
     //  parameter Real[__n_rollers] y = {rollerDirs[i,2]*R1 for i in 1:__n_rollers}
     //    "Roller y_0";
     //  parameter Real[__n_rollers] z = {rollerDirs[i,3]*R1 for i in 1:__n_rollers}
     //    "Roller z_0";
     //  parameter Real[3,3] nhlTotalWheelInertia = diagonal(wheelInertia) + sum(( transpose(roller_T0s[i,:,:])*diagonal(rollerInertia)*roller_T0s[i,:,:] +
     //                                                                            rollerMass * [ x[i]^2,                     -(x[i]^2+y[i]^2+x[i]*y[i]), -(x[i]^2+z[i]^2+x[i]*z[i]);
     //                                                                                           -(x[i]^2+y[i]^2+x[i]*y[i]), y[i]^2,                     -(y[i]^2+z[i]^2+y[i]*z[i]);
     //                                                                                           -(x[i]^2+z[i]^2+x[i]*z[i]), -(y[i]^2+z[i]^2+y[i]*z[i]), z[i]^2])                        for i in 1:__n_rollers);
       parameter Real nhlTotalWheelAxisInertia = wheelInertia[1] + 3*((wheelRadius*cos(pi/__n_rollers))^2)*rollerInertia[1];
     
       parameter Real _psi = pi/6;
     
       Real nhl_x;
       Real nhl_y;
       Real nhl_angle;
       Real[3] nhl_wheelAngles;
     
       Real mbs_x;
       Real mbs_y;
       Real mbs_angle;
       Real[3] mbs_wheelAngles;
     
       Real[3] mbs_vt;
       Real mbs_platform_r2;
       Real[3] mbs_omega;
       Real[3] mbs_rollerInContactNumber;
       Real[3,__n_rollers] mbs_contacts_h;
       Real[3] mbs_forceTsqrt;
     
       Real delta_x;
       Real delta_y;
       Real delta_angle;
       Real[3] delta_wheelAngles;
     
       NonHolonomicOmniPlatform nhl(
          psi = _psi,
          __0_v =  {__0_v_x, __0_v_y},
          __0_omega = __0_omega,
          m0 = platformMass,
          mi = {wheelMass for i in 1:3},
          Ii = {nhlTotalWheelAxisInertia for i in 1:3},
          Iiw = {wheelInertia[2] for i in 1:3},
          I0 = platformInertia);
     //     Ii = {wheelInertia[1] for i in 1:3},
     
       PointContact.NRollersPointContactOmniWheelSetTestGeneral mbs(
          psi = _psi,
          v0 = {__0_v_x, 0, __0_v_y},
          om0 = __0_omega,
          rollerFraction = __rollerFraction,
          n = __n_rollers,
          platformDiameter = platformDiameter,
          wheelRadius = wheelRadius,
          wheelMass = wheelMass,
          platformMass = platformMass,
          wheelInertia = {wheelInertia[2], wheelInertia[2], wheelInertia[1]},
          platformInertia = platformInertia);
     
     equation
       nhl_x     = nhl.x;
       nhl_y     = nhl.y;
       nhl_angle = nhl.phi*180/pi;
       nhl_wheelAngles = nhl.wheel_angle*180/pi;
     
       mbs_x     = mbs._r1;
       mbs_y     = mbs._r3;
       mbs_angle = mbs._angle0;
       mbs_wheelAngles = {mbs._angle1, mbs._angle2, mbs._angle3};
       mbs_vt = {mbs.Wheel1._vt, mbs.Wheel2._vt, mbs.Wheel3._vt};
       mbs_platform_r2 = mbs.Platform.r[2];
       mbs_omega = mbs.Platform.omega;
       mbs_rollerInContactNumber = {mbs.Wheel1._rollerInContactNumber, mbs.Wheel2._rollerInContactNumber, mbs.Wheel3._rollerInContactNumber};
       mbs_contacts_h = {{mbs.Wheel1.Contacts[i].h for i in 1:__n_rollers},
                         {mbs.Wheel2.Contacts[i].h for i in 1:__n_rollers},
                         {mbs.Wheel3.Contacts[i].h for i in 1:__n_rollers}};
       mbs_forceTsqrt = {sum(mbs.Wheel1.Contacts[i].ForceTsqrt*mbs.Wheel1.Contacts[i].isInContact for i in 1:__n_rollers),
                         sum(mbs.Wheel2.Contacts[i].ForceTsqrt*mbs.Wheel2.Contacts[i].isInContact for i in 1:__n_rollers),
                         sum(mbs.Wheel3.Contacts[i].ForceTsqrt*mbs.Wheel3.Contacts[i].isInContact for i in 1:__n_rollers)};
     
       delta_x     = mbs_x - nhl_x;
       delta_y     = mbs_y - nhl_y;
       delta_angle = mbs_angle - nhl_angle;
       delta_wheelAngles = mbs_wheelAngles - nhl_wheelAngles;
     
       annotation (experiment(StopTime=10, Tolerance=1e-007),
           __Dymola_experimentSetupOutput);
     end ComparisonBorisovAndMbsGeneral;
     
     model ComparisonMbsGeneralAndBorisov
       "Compares OmniWheelSet physical MBS model with Borisov non-holonomic."
       import Modelica.Constants.pi;
     
       parameter Real __0_v_x =             0;
       parameter Real __0_v_y =             0;
       parameter Real __0_omega =           1;
       parameter Integer __n_rollers =      4;
       parameter Real __rollerFraction =    1e-1;
     
       parameter Real platformDiameter = 1;
       parameter Real wheelMass =        1;
       parameter Real platformMass =     3.3;
       parameter Real wheelRadius =      1;
       parameter Real wheelAspectRatio = 1/5 "Thickness/Radius";
       parameter Real[3] wheelInertia =  wheelMass * (wheelRadius^2) * {1/2, 1/4 + 1/12*wheelAspectRatio^2, 1/4 + 1/12*wheelAspectRatio^2};
       //MBS prokidyvaet whIn stupize, a rolleram delaet individ.
       parameter Real platformInertia =  platformMass * ((platformDiameter/2)^2) / 2;
     
     //  parameter Real[3] rollerInertia = rollerMass * {1/2*(R-R1)^2, 3/12*(R-R1)^2 + 1/12*(2*R*sin(alpha))^2, 3/12*(R-R1)^2 + 1/12*(2*R*sin(alpha))^2};
     
       //should duplicate same from Wheel class:
       parameter Real rollerMass = wheelMass*__rollerFraction/__n_rollers;
     //  parameter Real alpha = pi/__n_rollers "Max angle of the half-sector";
     //  parameter Real[__n_rollers,4] roller_q0s_local = {{cos(-alpha*(i-1)), 0, 0, -1 * sin(-alpha*(i-1))} for i in 1:__n_rollers};
     //  parameter Real[__n_rollers,4] roller_q0s =       {QMult1({1,0,0,0}, roller_q0s_local[i,:]) for i in 1:__n_rollers};
       parameter Real[3] rollerInertia = rollerMass * {1/2, 1, 1};
     //  parameter Real R1 = wheelRadius*cos(alpha) "Omni wheel inner radius";
     //  parameter Real[__n_rollers,3] rollerDirs = {{cos(-pi/2 + 2*alpha*(i-1)), sin(-pi/2 + 2*alpha*(i-1)), 0} for i in 1:__n_rollers};
     //  parameter Real[__n_rollers,3,3] roller_T0s = {QToT(roller_q0s[i,:]) for i in 1:__n_rollers};
     //  parameter Real[__n_rollers] x = {rollerDirs[i,1]*R1 for i in 1:__n_rollers}
     //    "Roller x_0";
     //  parameter Real[__n_rollers] y = {rollerDirs[i,2]*R1 for i in 1:__n_rollers}
     //    "Roller y_0";
     //  parameter Real[__n_rollers] z = {rollerDirs[i,3]*R1 for i in 1:__n_rollers}
     //    "Roller z_0";
     //  parameter Real[3,3] nhlTotalWheelInertia = diagonal(wheelInertia) + sum(( transpose(roller_T0s[i,:,:])*diagonal(rollerInertia)*roller_T0s[i,:,:] +
     //                                                                            rollerMass * [ x[i]^2,                     -(x[i]^2+y[i]^2+x[i]*y[i]), -(x[i]^2+z[i]^2+x[i]*z[i]);
     //                                                                                           -(x[i]^2+y[i]^2+x[i]*y[i]), y[i]^2,                     -(y[i]^2+z[i]^2+y[i]*z[i]);
     //                                                                                           -(x[i]^2+z[i]^2+x[i]*z[i]), -(y[i]^2+z[i]^2+y[i]*z[i]), z[i]^2])                        for i in 1:__n_rollers);
       parameter Real nhlTotalWheelAxisInertia = wheelInertia[1] + 3*((wheelRadius*cos(pi/__n_rollers))^2)*rollerInertia[1];
     
       parameter Real _psi = 0;
     
       NonHolonomicOmniPlatform nhl(
          psi = _psi,
          __0_v =  {__0_v_x, __0_v_y},
          __0_omega = __0_omega,
          m0 = platformMass,
          mi = {wheelMass for i in 1:3},
          Ii = {nhlTotalWheelAxisInertia for i in 1:3},
          Iiw = {wheelInertia[2] for i in 1:3},
          I0 = platformInertia);
     //     Ii = {wheelInertia[1] for i in 1:3},
     
       PointContact.NRollersPointContactOmniWheelSetTestGeneral mbs(
          v0 = {__0_v_x, 0, __0_v_y},
          om0 = __0_omega,
          rollerFraction = __rollerFraction,
          n = __n_rollers,
          psi = _psi,
          platformDiameter = platformDiameter,
          wheelRadius = wheelRadius,
          wheelMass = wheelMass,
          platformMass = platformMass,
          wheelInertia = {wheelInertia[2], wheelInertia[2], wheelInertia[1]},
          platformInertia = platformInertia);
     
       annotation (experiment(StopTime=10, Tolerance=1e-007),
           __Dymola_experimentSetupOutput);
     end ComparisonMbsGeneralAndBorisov;

end Core;
