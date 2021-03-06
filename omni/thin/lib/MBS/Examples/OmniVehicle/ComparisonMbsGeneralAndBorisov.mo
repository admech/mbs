within MBS.Examples.OmniVehicle;

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
