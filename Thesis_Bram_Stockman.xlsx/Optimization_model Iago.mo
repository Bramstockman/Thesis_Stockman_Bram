within GIT_thesis_Bram_Stockman;
model Optimization_model "Model fro MPC optimization"
     package Medium = IDEAS.Media.Water;
  Real T_20 = 20;
  Real T_26 = 26;

  UpscaleCase900                      upscaleCase900_HVAC(redeclare package
      Medium = Borefield.Control.Media.DryAir)
    annotation (Placement(transformation(extent={{90,-168},{120,-148}})));
  IDEAS.Fluid.HeatExchangers.RadiantSlab.EmbeddedPipe embeddedPipe(
    redeclare package Medium = Medium,
    A_floor=1200,
    redeclare IDEAS.Fluid.HeatExchangers.RadiantSlab.BaseClasses.FH_Standard1
      RadSlaCha,
    m_flow_nominal=idealSource_embeddedPipe.m_flow_nominal,
    allowFlowReversal=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    T_start=308.15) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={144,-116})));
  IDEAS.Fluid.Movers.BaseClasses.IdealSource idealSource_embeddedPipe(
    redeclare package Medium = Medium,
    allowFlowReversal=false,
    control_m_flow=true,
    control_dp=false) annotation (Placement(transformation(
        extent={{-8,-8},{8,8}},
        rotation=270,
        origin={232,-100})));
  IDEAS.Fluid.Actuators.Valves.Simplified.ThreeWayValveMotor
    SwitchHeatCoolEmbeddedPipe(
    redeclare package Medium = Medium,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    m_flow_nominal=idealSource_embeddedPipe.m_flow_nominal,
    T_start=308.15) annotation (Placement(transformation(
        extent={{-8,-8},{8,8}},
        rotation=270,
        origin={232,14})));
  Borefield.Control.Fluid.HeatPumps.HeatPump_y heatPump_y(
    redeclare package Medium1 = Medium,
    redeclare package Medium2 = Medium,
    m2_flow_nominal=idealSource_BorFie.m_flow_nominal,
    dp1_nominal=0,
    dp2_nominal=0,
    Q_nom=36000,
    m1_flow_nominal=idealSource_embeddedPipe.m_flow_nominal) annotation (
      Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=270,
        origin={54,52})));

  Borefield.Control.Fluid.Geothermal.Borefields.OneUTube
                                             oneUTube(
                                                    redeclare package Medium =
        Medium, borFieDat=
        IDEAS.Fluid.Geothermal.Borefields.Data.Borefield.Example(
          filDat=IDEAS.Fluid.Geothermal.Borefields.Data.Filling.Bentonite(
            kFil=0.6,
            cFil=1650,
            dFil=1000,
            steadyState=true),
          soiDat=IDEAS.Fluid.Geothermal.Borefields.Data.Soil.SandStone(
            kSoi=2.2,
            cSoi=2470,
            dSoi=1000),
          conDat=IDEAS.Fluid.Geothermal.Borefields.Data.Configuration.Example(
            borCon=IDEAS.Fluid.Geothermal.Borefields.Types.BoreholeConfiguration.SingleUTube,
            use_Rb=true,
            Rb=0.266,
            mBor_flow_nominal=6.5/18,
            mBorFie_flow_nominal=6.5,
            hBor=100,
            dBor=1,
            nBor=18,
            cooBor={{0,0},{0,6},{6,0},{6,6},{0,12},{6,12},{12,12},{12,6},{12,
            0},{0,18},{6,18},{12,18},{18,18},{18,12},{18,6},{18,0},{0,24},{6,
            24}},
            kTub=0.38)),
    allowFlowReversal=false,
    from_dp=true,
    linearizeFlowResistance=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-20,-28})));
  IDEAS.Fluid.HeatExchangers.ConstantEffectiveness hex(
    redeclare package Medium1 = Medium,
    redeclare package Medium2 = Medium,
    eps=0.8,
    m1_flow_nominal=idealSource_BorFie.m_flow_nominal,
    dp1_nominal=0,
    dp2_nominal=0,
    m2_flow_nominal=idealSource_embeddedPipe.m_flow_nominal,
    allowFlowReversal1=false,
    allowFlowReversal2=false,
    linearizeFlowResistance1=true,
    linearizeFlowResistance2=true) annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=270,
        origin={54,-2})));
  IDEAS.Fluid.Actuators.Valves.Simplified.ThreeWayValveMotor
    SwitchHeatCoolBorefield(
    redeclare package Medium = Medium,
    m_flow_nominal=idealSource_BorFie.m_flow_nominal,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState) annotation (
      Placement(transformation(
        extent={{8,-8},{-8,8}},
        rotation=90,
        origin={-60,14})));
  IDEAS.Fluid.Sources.Boundary_pT bou(
    redeclare package Medium = Medium,
    p=150000,
    nPorts=1)
    annotation (Placement(transformation(extent={{-82,70},{-66,86}})));
  IDEAS.Fluid.Movers.BaseClasses.IdealSource idealSource_BorFie(
    redeclare package Medium = Medium,
    allowFlowReversal=false,
    control_m_flow=true,
    control_dp=false) annotation (Placement(transformation(
        extent={{-9,-8},{9,8}},
        rotation=90,
        origin={-60,-15})));
  IDEAS.Fluid.Sources.Boundary_pT bou1(
    redeclare package Medium = Medium,
    p=150000,
    nPorts=1)
    annotation (Placement(transformation(extent={{-7,-7},{7,7}},
        rotation=180,
        origin={251,79})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort EmbeddedPipeIn(
    redeclare package Medium = Medium,
    tau=0,
    initType=Modelica.Blocks.Types.Init.SteadyState,
    m_flow_nominal=idealSource_embeddedPipe.m_flow_nominal,
    T_start=308.15)
    annotation (Placement(transformation(extent={{190,-110},{176,-122}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort EmbeddedPipeOut(
    redeclare package Medium = Medium,
    tau=0,
    initType=Modelica.Blocks.Types.Init.SteadyState,
    m_flow_nominal=idealSource_embeddedPipe.m_flow_nominal,
    T_start=308.15)
    annotation (Placement(transformation(extent={{112,-110},{98,-122}})));
  IDEAS.Fluid.Actuators.Valves.Simplified.ThreeWayValveMotor
    threeWayValveTempSetoint(
    redeclare package Medium = Medium,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    allowFlowReversal=true,
    m_flow_nominal=idealSource_embeddedPipe.m_flow_nominal,
    T_start=308.15) annotation (Placement(transformation(
        extent={{-8,8},{8,-8}},
        rotation=270,
        origin={80,-58})));
  IDEAS.Fluid.FixedResistances.Junction jun(
    redeclare package Medium = Medium,
    dp_nominal={0,0,0},
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    m_flow_nominal={idealSource_embeddedPipe.m_flow_nominal,
        idealSource_embeddedPipe.m_flow_nominal,idealSource_embeddedPipe.m_flow_nominal},
    portFlowDirection_1=Modelica.Fluid.Types.PortFlowDirection.Entering,
    portFlowDirection_2=Modelica.Fluid.Types.PortFlowDirection.Leaving,
    portFlowDirection_3=Modelica.Fluid.Types.PortFlowDirection.Entering,
    linearized=true,
    T_start=308.15) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={232,-58})));

  IDEAS.Fluid.FixedResistances.Junction jun1(
    redeclare package Medium = Medium,
    dp_nominal={0,0,0},
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    m_flow_nominal={idealSource_BorFie.m_flow_nominal,idealSource_BorFie.m_flow_nominal,
        idealSource_BorFie.m_flow_nominal},
    portFlowDirection_1=Modelica.Fluid.Types.PortFlowDirection.Entering,
    portFlowDirection_2=Modelica.Fluid.Types.PortFlowDirection.Leaving,
    portFlowDirection_3=Modelica.Fluid.Types.PortFlowDirection.Entering,
    linearized=true) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={24,-28})));
  IDEAS.Fluid.HeatExchangers.HeaterCooler_u AuxiliaryHeater1(
    Q_flow_nominal=36000,
    redeclare package Medium = Medium,
    dp_nominal=0,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    m_flow_nominal=idealSource_embeddedPipe.m_flow_nominal)
    annotation (Placement(transformation(extent={{140,70},{160,90}})));
  Modelica.Blocks.Interfaces.RealInput Threewayvalve_HeatingCooling(
    min=0,
    max=1,
    start=1,
    nominal=1) " 1 = Heating, 0=Cooling"
    annotation (Placement(transformation(extent={{-216,22},{-176,62}})));
  Modelica.Blocks.Interfaces.RealInput Fan_BorFie(
    final unit="kg/s",
    min=0,
    max=1,
    start=1,
    nominal=1)
    annotation (Placement(transformation(extent={{-214,-40},{-174,0}})));
  Modelica.Blocks.Interfaces.RealInput Threewayvalve_TemperatureSetpoint(
    min=0,
    max=1,
    start=1,
    nominal=1)
    annotation (Placement(transformation(extent={{-214,-78},{-174,-38}})));
  Modelica.Blocks.Interfaces.RealInput Fan_EmbeddedPipe(
    final unit="kg/s",
    min=0,
    max=1,
    start=1,
    nominal=1)
    annotation (Placement(transformation(extent={{-218,-254},{-178,-214}})));
  Modelica.Blocks.Interfaces.RealInput HeaPum(
    final unit="K",
    min=0,
    max=1,
    start=1,
    nominal=1)
    annotation (Placement(transformation(extent={{-216,90},{-176,130}})));
  Modelica.Blocks.Interfaces.RealInput Perfect_Heater(
    final unit="K",
    min=0,
    max=1,
    start=1,
    nominal=1)
    annotation (Placement(transformation(extent={{-216,126},{-176,166}})));
  Modelica.Blocks.Sources.Constant const(k=20*120*0.3 + 1200*0.9)
    annotation (Placement(transformation(extent={{-48,-166},{-28,-146}})));
  IDEAS.Controls.SetPoints.OccupancySchedule occSch(
    occupancy=3600*{7,17,31,41,55,65,79,89,103,113,127,137,151,161},
    period=604800,
    firstEntryOccupied=false)
    annotation (Placement(transformation(extent={{-48,-132},{-28,-112}})));
  Modelica.Blocks.Logical.Switch switch3
    annotation (Placement(transformation(extent={{-4,-158},{16,-138}})));
  Modelica.Blocks.Sources.Constant const6(k=70*0.3*20)
    annotation (Placement(transformation(extent={{-48,-100},{-28,-80}})));
  Modelica.Blocks.Sources.Constant const1(k=20*120*0.7 + 1200*0.1)
    annotation (Placement(transformation(extent={{-118,-210},{-98,-190}})));
  IDEAS.Controls.SetPoints.OccupancySchedule occSch1(
    occupancy=3600*{7,17,31,41,55,65,79,89,103,113,127,137,151,161},
    period=604800,
    firstEntryOccupied=false)
    annotation (Placement(transformation(extent={{-118,-176},{-98,-156}})));
  Modelica.Blocks.Logical.Switch switch1
    annotation (Placement(transformation(extent={{-74,-202},{-54,-182}})));
  Modelica.Blocks.Sources.Constant const2(k=70*0.7*20)
    annotation (Placement(transformation(extent={{-118,-144},{-98,-124}})));
protected
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow preHeaFloCon(final
      alpha=0)
    annotation (Placement(transformation(extent={{46,-198},{66,-178}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow preHeaFloRad(final
      alpha=0)
    annotation (Placement(transformation(extent={{44,-222},{64,-202}})));
equation
  connect(upscaleCase900_HVAC.heatPortEmb, embeddedPipe.heatPortEmb)
    annotation (Line(points={{120,-152},{144,-152},{144,-126}},
                                                           color={191,0,0}));
  connect(idealSource_BorFie.port_a, oneUTube.port_b) annotation (Line(points={{
          -60,-24},{-60,-28},{-30,-28}}, color={0,127,255}));
  connect(SwitchHeatCoolBorefield.port_b, idealSource_BorFie.port_b)
    annotation (Line(points={{-60,6},{-60,-6}}, color={0,127,255}));
  connect(SwitchHeatCoolBorefield.port_a2, hex.port_a1)
    annotation (Line(points={{-52,14},{48,14},{48,8}},
                                               color={0,127,255}));
  connect(embeddedPipe.port_a, EmbeddedPipeIn.port_b)
    annotation (Line(points={{154,-116},{176,-116}},
                                                 color={0,127,255}));
  connect(idealSource_embeddedPipe.port_b, EmbeddedPipeIn.port_a) annotation (
      Line(points={{232,-108},{232,-116},{190,-116}}, color={0,127,255}));
  connect(embeddedPipe.port_b, EmbeddedPipeOut.port_a)
    annotation (Line(points={{134,-116},{112,-116}},
                                                 color={0,127,255}));
  connect(threeWayValveTempSetoint.port_b, EmbeddedPipeOut.port_b)
    annotation (Line(points={{80,-66},{80,-116},{98,-116}},
        color={0,127,255}));
  connect(threeWayValveTempSetoint.port_a2, jun.port_3) annotation (Line(
        points={{88,-58},{222,-58}},                  color={0,127,255}));
  connect(SwitchHeatCoolEmbeddedPipe.port_b, jun.port_1)
    annotation (Line(points={{232,6},{232,-48}},
                                               color={0,127,255}));
  connect(idealSource_embeddedPipe.port_a, jun.port_2)
    annotation (Line(points={{232,-92},{232,-68}}, color={0,127,255}));
  connect(oneUTube.port_a, jun1.port_2)
    annotation (Line(points={{-10,-28},{14,-28}}, color={0,127,255}));
  connect(hex.port_b1, jun1.port_1) annotation (Line(points={{48,-12},{48,-28},{
          34,-28}},           color={0,127,255}));
  connect(threeWayValveTempSetoint.port_a1, hex.port_a2) annotation (Line(
        points={{80,-50},{60,-50},{60,-12}},        color={0,127,255}));

  connect(SwitchHeatCoolBorefield.port_a1, heatPump_y.port_a2) annotation (Line(
        points={{-60,22},{-60,78},{48,78},{48,62}}, color={0,127,255}));
  connect(heatPump_y.port_b2, jun1.port_3) annotation (Line(points={{48,42},{48,
          26},{24,26},{24,-18}}, color={0,127,255}));
  connect(bou.ports[1], heatPump_y.port_a2)
    annotation (Line(points={{-66,78},{48,78},{48,62}}, color={0,127,255}));
  connect(threeWayValveTempSetoint.port_a1, heatPump_y.port_a1) annotation (
      Line(points={{80,-50},{80,26},{60,26},{60,42}}, color={0,127,255}));
  connect(hex.port_b2, SwitchHeatCoolEmbeddedPipe.port_a2)
    annotation (Line(points={{60,8},{60,14},{224,14}}, color={0,127,255}));
  connect(heatPump_y.port_b1, AuxiliaryHeater1.port_a)
    annotation (Line(points={{60,62},{60,80},{140,80}}, color={0,127,255}));
  connect(AuxiliaryHeater1.port_b, SwitchHeatCoolEmbeddedPipe.port_a1)
    annotation (Line(points={{160,80},{232,80},{232,22}}, color={0,127,255}));
  connect(bou1.ports[1], SwitchHeatCoolEmbeddedPipe.port_a1)
    annotation (Line(points={{244,79},{232,79},{232,22}}, color={0,127,255}));
  connect(heatPump_y.y, HeaPum) annotation (Line(points={{63,42},{63,32},{80,32},
          {80,110},{-196,110}}, color={0,0,127}));
  connect(Perfect_Heater, AuxiliaryHeater1.u) annotation (Line(points={{-196,146},
          {130,146},{130,86},{138,86}}, color={0,0,127}));
  connect(Threewayvalve_HeatingCooling, SwitchHeatCoolBorefield.ctrl)
    annotation (Line(points={{-196,42},{-100,42},{-100,14},{-68.64,14}}, color={
          0,0,127}));
  connect(SwitchHeatCoolEmbeddedPipe.ctrl, SwitchHeatCoolBorefield.ctrl)
    annotation (Line(points={{240.64,14},{276,14},{276,156},{-100,156},{-100,14},
          {-68.64,14}}, color={0,0,127}));
  connect(Fan_BorFie, idealSource_BorFie.m_flow_in) annotation (Line(points={{-194,
          -20},{-174,-20},{-174,-20.4},{-66.4,-20.4}}, color={0,0,127}));
  connect(threeWayValveTempSetoint.ctrl, Threewayvalve_TemperatureSetpoint)
    annotation (Line(points={{71.36,-58},{-194,-58}}, color={0,0,127}));
  connect(Fan_EmbeddedPipe, idealSource_embeddedPipe.m_flow_in) annotation (
      Line(points={{-198,-234},{260,-234},{260,-95.2},{238.4,-95.2}}, color={0,0,
          127}));
  connect(switch3.u2,occSch. occupied) annotation (Line(points={{-6,-148},{
          -16,-148},{-16,-128},{-27,-128}},    color={255,0,255}));
  connect(const6.y,switch3. u1) annotation (Line(points={{-27,-90},{-12,-90},
          {-12,-140},{-6,-140}},          color={0,0,127}));
  connect(switch3.u3, const.y)
    annotation (Line(points={{-6,-156},{-27,-156}}, color={0,0,127}));
  connect(switch3.y, preHeaFloCon.Q_flow) annotation (Line(points={{17,-148},
          {29.5,-148},{29.5,-188},{46,-188}}, color={0,0,127}));
  connect(switch1.u2, occSch1.occupied) annotation (Line(points={{-76,-192},{
          -86,-192},{-86,-172},{-97,-172}}, color={255,0,255}));
  connect(const2.y,switch1. u1) annotation (Line(points={{-97,-134},{-82,-134},
          {-82,-184},{-76,-184}},         color={0,0,127}));
  connect(switch1.u3, const1.y)
    annotation (Line(points={{-76,-200},{-97,-200}}, color={0,0,127}));
  connect(switch1.y, preHeaFloRad.Q_flow) annotation (Line(points={{-53,-192},
          {-4,-192},{-4,-212},{44,-212}}, color={0,0,127}));
  connect(preHeaFloCon.port, upscaleCase900_HVAC.heatPortCon[1]) annotation (
      Line(points={{66,-188},{136,-188},{136,-156},{120,-156}}, color={191,0,
          0}));
  connect(preHeaFloRad.port, upscaleCase900_HVAC.heatPortRad[1]) annotation (
      Line(points={{64,-212},{146,-212},{146,-160},{120,-160}}, color={191,0,
          0}));
  annotation (Diagram(coordinateSystem(extent={{-180,-240},{340,160}})), Icon(
        coordinateSystem(extent={{-180,-240},{340,160}})));
end Optimization_model;
