within GIT_thesis_Bram_Stockman;
model RBC_model_ASHRAE
     package Medium = IDEAS.Media.Water;
  Real T_20 = 20;
  Real T_26 = 26;

  UpscaleCase900                      upscaleCase900_HVAC
    annotation (Placement(transformation(extent={{-50,-180},{-20,-160}})));
  IDEAS.Fluid.HeatExchangers.RadiantSlab.EmbeddedPipe embeddedPipe(
    redeclare package Medium = Medium,
    A_floor=1200,
    redeclare IDEAS.Fluid.HeatExchangers.RadiantSlab.BaseClasses.FH_Standard1
      RadSlaCha,
    m_flow_nominal=fanEmbeddedPipe.m_flow_nominal,
    allowFlowReversal=false,
    T_start=308.15)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={74,-98})));
  IDEAS.Fluid.Movers.FlowControlled_m_flow fanEmbeddedPipe(
    redeclare package Medium = Medium,
    addPowerToMedium=false,
    tau=60,
    constantMassFlowRate=1,
    inputType=IDEAS.Fluid.Types.InputType.Continuous,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    use_inputFilter=false,
    allowFlowReversal=false,
    m_flow_nominal=1.6,
    T_start=308.15) annotation (Placement(transformation(
        extent={{-8,-8},{8,8}},
        rotation=270,
        origin={164,-82})));
  IDEAS.Fluid.MixingVolumes.MixingVolume BufferTank(
    redeclare package Medium = Medium,
    nPorts=7,
    m_flow_nominal=fanBorefield.m_flow_nominal,
    T_start=308.15,
    V=2)            annotation (Placement(transformation(
        extent={{-27,-27},{27,27}},
        rotation=0,
        origin={57,89})));
  IDEAS.Fluid.Actuators.Valves.Simplified.ThreeWayValveSwitch SwitchHeatCoolEmbeddedPipe(
    redeclare package Medium = Medium,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    m_flow_nominal=fanEmbeddedPipe.m_flow_nominal,
    T_start=308.15) annotation (Placement(transformation(
        extent={{-8,-8},{8,8}},
        rotation=270,
        origin={60,-4})));
  IDEAS.Fluid.Movers.FlowControlled_m_flow fanBuffertank(
    redeclare package Medium = Medium,
    addPowerToMedium=false,
    tau=60,
    constantMassFlowRate=1,
    inputType=IDEAS.Fluid.Types.InputType.Continuous,
    use_inputFilter=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    allowFlowReversal=false,
    m_flow_nominal=2,
    T_start=308.15) annotation (Placement(transformation(
        extent={{-8,-8},{8,8}},
        rotation=0,
        origin={28,52})));
  IDEAS.Fluid.HeatPumps.ScrollWaterToWater heaPum(
    redeclare package Medium1 = Medium,
    redeclare package Medium2 = Medium,
    allowFlowReversal1=false,
    allowFlowReversal2=false,
    redeclare package ref = IDEAS.Media.Refrigerants.R410A,
    datHeaPum=
        IDEAS.Fluid.HeatPumps.Data.ScrollWaterToWater.Heating.ClimateMaster_TMW036_12kW_4_90COP_R410A(),
    m1_flow_nominal=fanBuffertank.m_flow_nominal,
    m2_flow_nominal=fanBorefield.m_flow_nominal,
    dp1_nominal=0,
    dp2_nominal=0,
    enable_variable_speed=true,
    scaling_factor=3) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=270,
        origin={-20,42})));

  IDEAS.Fluid.Geothermal.Borefields.OneUTube borFie(redeclare package Medium =
        Medium,
    allowFlowReversal=false,
    from_dp=true,
    linearizeFlowResistance=false,
    borFieDat=IDEAS.Fluid.Geothermal.Borefields.Data.Borefield.Example(
          filDat=IDEAS.Fluid.Geothermal.Borefields.Data.Filling.Bentonite(
            kFil=1.6,
            cFil=1200,
            dFil=1340,
            steadyState=true),
          soiDat=IDEAS.Fluid.Geothermal.Borefields.Data.Soil.SandStone(
            kSoi=1.9,
            cSoi=1364,
            dSoi=1760),
          conDat=IDEAS.Fluid.Geothermal.Borefields.Data.Configuration.Example(
            borCon=IDEAS.Fluid.Geothermal.Borefields.Types.BoreholeConfiguration.SingleUTube,
            use_Rb=true,
            Rb=0.0872,
            mBor_flow_nominal=1.9/4,
            mBorFie_flow_nominal=1.9,
            hBor=154,
            rBor=0.150,
            dBor=1,
            nBor=16,
            cooBor=[0,0; 0,15.4; 0,30.8; 0,46.2; 15.4,0; 15.4,15.4; 15.4,30.8;
            15.4,46.2; 30.8,0; 30.8,15.4; 30.8,30.8; 30.8,46.2; 46.2,0; 46.2,
            15.4; 46.2,30.8; 46.2,46.2],
            rTub=0.02,
            kTub=0.43,
            eTub=0.0036,
            xC=0.11)))
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-54,-42})));
  IDEAS.Fluid.HeatExchangers.ConstantEffectiveness hex(
    redeclare package Medium1 = Medium,
    redeclare package Medium2 = Medium,
    eps=0.8,
    m1_flow_nominal=fanBorefield.m_flow_nominal,
    dp1_nominal=0,
    dp2_nominal=0,
    m2_flow_nominal=fanEmbeddedPipe.m_flow_nominal,
    allowFlowReversal1=false,
    allowFlowReversal2=false,
    linearizeFlowResistance1=true,
    linearizeFlowResistance2=true)
                       annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=270,
        origin={4,-14})));
  IDEAS.Fluid.Actuators.Valves.Simplified.ThreeWayValveSwitch
    SwitchHeatCoolBorefield(
    redeclare package Medium = Medium,
    m_flow_nominal=fanBorefield.m_flow_nominal,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState) annotation (
      Placement(transformation(
        extent={{8,-8},{-8,8}},
        rotation=90,
        origin={-78,-4})));
  IDEAS.Fluid.Sources.Boundary_pT bou(
    redeclare package Medium = Medium,
    p=150000,
    nPorts=1)
    annotation (Placement(transformation(extent={{-106,64},{-90,80}})));
  IDEAS.Fluid.Movers.FlowControlled_m_flow fanBorefield(
    redeclare IDEAS.Fluid.Movers.Data.Generic per,
    redeclare package Medium = Medium,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    addPowerToMedium=false,
    tau=60,
    use_inputFilter=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    inputType=IDEAS.Fluid.Types.InputType.Continuous,
    allowFlowReversal=false,
    m_flow_nominal=1.9)
                      annotation (Placement(transformation(
        extent={{-9,-8},{9,8}},
        rotation=90,
        origin={-78,-27})));
  IDEAS.Fluid.Sources.Boundary_pT bou1(
    redeclare package Medium = Medium,
    nPorts=1,
    p=150000)
    annotation (Placement(transformation(extent={{-7,-7},{7,7}},
        rotation=180,
        origin={85,13})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
    annotation (Placement(transformation(extent={{20,122},{34,136}})));
  Modelica.Blocks.Math.RealToBoolean Heating
    annotation (Placement(transformation(extent={{-288,114},{-276,126}})));
  Modelica.Blocks.Logical.Not not1
    annotation (Placement(transformation(extent={{-248,114},{-236,126}})));
  Modelica.Blocks.Math.BooleanToReal booleanToReal
    annotation (Placement(transformation(extent={{298,20},{312,34}})));
  Modelica.Blocks.Logical.Or or1
    annotation (Placement(transformation(extent={{-194,42},{-180,56}})));
  Modelica.Blocks.Math.Gain gain(k=fanBuffertank.m_flow_nominal)
                                      annotation (Placement(transformation(
        extent={{-8,-8},{8,8}},
        rotation=270,
        origin={-4,124})));
  Modelica.Blocks.Math.BooleanToReal booleanToReal1
    annotation (Placement(transformation(extent={{-172,42},{-158,56}})));
  Modelica.Blocks.Math.Gain gain1(k=fanBorefield.m_flow_nominal)
                                       annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=0,
        origin={-139,49})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort EmbeddedPipeIn(
    redeclare package Medium = Medium,
    tau=0,
    initType=Modelica.Blocks.Types.Init.SteadyState,
    m_flow_nominal=fanEmbeddedPipe.m_flow_nominal,
    T_start=308.15)
    annotation (Placement(transformation(extent={{118,-92},{104,-104}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort EmbeddedPipeOut(
    redeclare package Medium = Medium,
    tau=0,
    initType=Modelica.Blocks.Types.Init.SteadyState,
    m_flow_nominal=fanEmbeddedPipe.m_flow_nominal,
    T_start=308.15)
    annotation (Placement(transformation(extent={{46,-92},{32,-104}})));
  IDEAS.Fluid.Actuators.Valves.Simplified.ThreeWayValveMotor threeWayValveTempSetoint(
    redeclare package Medium = Medium,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    allowFlowReversal=true,
    m_flow_nominal=fanEmbeddedPipe.m_flow_nominal,
    T_start=308.15) annotation (Placement(transformation(
        extent={{-8,8},{8,-8}},
        rotation=270,
        origin={26,-54})));
  IDEAS.Controls.Continuous.LimPID conPIDOutHeat(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    xi_start=0,
    yMax=1,
    yMin=0.1,
    reverseAction=true,
    y_reset=0,
    k=1,
    Ti=60)
    annotation (Placement(transformation(extent={{218,-222},{234,-206}})));
  IDEAS.Controls.Continuous.LimPID conPIDINHeat(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    yMax=1,
    xi_start=1,
    k=1,
    yMin=0,
    reverseAction=false,
    Ti=200)
    annotation (Placement(transformation(extent={{-172,-70},{-156,-54}})));
  RunningMeanTemperature6h                      runningMeanTemperature6h
    annotation (Placement(transformation(extent={{-258,-88},{-240,-68}})));
  IDEAS.Controls.SetPoints.Table HeatingCurveIn(table=[-8.0 + 273,35 + 273;
        15 + 273,24 + 273])
    annotation (Placement(transformation(extent={{-214,-72},{-194,-52}})));
  IDEAS.Controls.SetPoints.Table HeatingCurveOut(table=[-7 + 273,4.85; 5 + 273,3;
        12 + 273,0])
    annotation (Placement(transformation(extent={{176,-224},{196,-204}})));
  IDEAS.Fluid.FixedResistances.Junction jun(
    redeclare package Medium = Medium,
    dp_nominal={0,0,0},
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    m_flow_nominal={fanEmbeddedPipe.m_flow_nominal,fanEmbeddedPipe.m_flow_nominal,
        fanEmbeddedPipe.m_flow_nominal},
    T_start=308.15,
    portFlowDirection_1=Modelica.Fluid.Types.PortFlowDirection.Entering,
    portFlowDirection_2=Modelica.Fluid.Types.PortFlowDirection.Leaving,
    portFlowDirection_3=Modelica.Fluid.Types.PortFlowDirection.Entering,
    linearized=true)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=270,
        origin={164,-54})));
  IDEAS.Fluid.FixedResistances.Junction jun1(
    redeclare package Medium = Medium,
    dp_nominal={0,0,0},
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    m_flow_nominal={fanBorefield.m_flow_nominal,fanBorefield.m_flow_nominal,
        fanBorefield.m_flow_nominal},
    portFlowDirection_1=Modelica.Fluid.Types.PortFlowDirection.Entering,
    portFlowDirection_2=Modelica.Fluid.Types.PortFlowDirection.Leaving,
    portFlowDirection_3=Modelica.Fluid.Types.PortFlowDirection.Entering,
    linearized=true)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-26,-42})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort HexOut(
    redeclare package Medium = Medium,
    tau=0,
    initType=Modelica.Blocks.Types.Init.SteadyState,
    m_flow_nominal=fanEmbeddedPipe.m_flow_nominal,
    T_start=308.15)
    annotation (Placement(transformation(extent={{30,2},{16,-10}})));
  RunningMeanTemperature6h                      runningMeanTemperature6h1
    annotation (Placement(transformation(extent={{136,-224},{154,-204}})));
  RunningMeanTemperature6h                      runningMeanTemperature6h2
    annotation (Placement(transformation(extent={{142,-270},{160,-250}})));
  IDEAS.Controls.Continuous.LimPID conPIDOutCool(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    xi_start=0,
    Ti=60,
    k=1,
    yMax=1,
    yMin=0.1,
    reverseAction=false)
    annotation (Placement(transformation(extent={{222,-268},{238,-252}})));
  Modelica.Blocks.Logical.Switch switch1
    annotation (Placement(transformation(extent={{306,-250},{326,-230}})));
  IDEAS.Controls.SetPoints.Table CoolingCurveIn(table=[22 + 273,22 + 273; 25
         + 273,17 + 273])
    annotation (Placement(transformation(extent={{-214,-130},{-194,-110}})));
  IDEAS.Controls.Continuous.LimPID conPIDINCool(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    yMax=1,
    xi_start=1,
    k=1,
    yMin=0,
    reverseAction=true,
    Ti=60)
    annotation (Placement(transformation(extent={{-174,-128},{-158,-112}})));
  Modelica.Blocks.Logical.Switch switch2
    annotation (Placement(transformation(extent={{-124,-106},{-104,-86}})));
  HeatingCoolingSet                      heatingCoolingSet1(HeatSet=19 + 273,
      CoolSet=20 + 273)
    annotation (Placement(transformation(extent={{-332,88},{-312,108}})));
  Modelica.Blocks.Math.RealToBoolean Cooling
    annotation (Placement(transformation(extent={{-292,70},{-278,84}})));
  Modelica.Blocks.Logical.Or  Neutral
    annotation (Placement(transformation(extent={{-248,92},{-236,104}})));
  Modelica.Blocks.Logical.Switch switch3
    annotation (Placement(transformation(extent={{-120,-34},{-104,-18}})));
  Modelica.Blocks.Sources.Constant const(k=0)
    annotation (Placement(transformation(extent={{-252,-40},{-238,-26}})));
  Modelica.Blocks.Logical.LogicalSwitch logicalSwitch
    annotation (Placement(transformation(extent={{266,-6},{282,10}})));
  Modelica.Blocks.Sources.BooleanConstant booleanConstant(k=false)
    annotation (Placement(transformation(extent={{236,-24},{250,-10}})));
  Modelica.Blocks.Logical.Switch switch4
    annotation (Placement(transformation(extent={{216,-74},{200,-58}})));
  Modelica.Blocks.Sources.Constant const1(k=0.01)
    annotation (Placement(transformation(extent={{254,-104},{238,-88}})));
  Modelica.Blocks.Math.Add add1(k1=+1, k2=-1)
    annotation (Placement(transformation(extent={{52,-286},{72,-266}})));
  IDEAS.Controls.SetPoints.Table CoolingCurveOut(table=[15 + 273,0; 20 + 273,-4;
        25 + 273,-6])
    annotation (Placement(transformation(extent={{180,-270},{200,-250}})));
  Comfort comfort
    annotation (Placement(transformation(extent={{-338,-304},{-320,-286}})));
  IDEAS.Controls.Continuous.LimPID conPIDHeatPump(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    xi_start=0,
    yMax=1,
    y_reset=0,
    k=1,
    Ti=60,
    yMin=0,
    reverseAction=false)
    annotation (Placement(transformation(extent={{244,66},{260,82}})));
  Modelica.Blocks.Sources.Constant const2(k=35 + 273)
    annotation (Placement(transformation(extent={{210,64},{230,84}})));
  Modelica.Blocks.Logical.Switch switch5
    annotation (Placement(transformation(extent={{288,60},{304,76}})));
  Modelica.Blocks.Sources.Constant const3(k=0)
    annotation (Placement(transformation(extent={{236,22},{256,42}})));
  Modelica.Blocks.Math.Add Lowerbound_error(k1=+1, k2=-1)
    annotation (Placement(transformation(extent={{-296,-284},{-276,-264}})));
  Modelica.Blocks.Math.Add Upperbound_error(k1=-1, k2=+1)
    annotation (Placement(transformation(extent={{-296,-340},{-276,-320}})));
  Modelica.Blocks.Logical.LessThreshold    lessThreshold
    annotation (Placement(transformation(extent={{-242,-284},{-222,-264}})));
  Modelica.Blocks.Logical.LessThreshold lessThreshold1
    annotation (Placement(transformation(extent={{-242,-340},{-222,-320}})));
  Modelica.Blocks.Logical.Switch switch6
    annotation (Placement(transformation(extent={{-140,-354},{-120,-334}})));
  Modelica.Blocks.Logical.Switch switch7
    annotation (Placement(transformation(extent={{-142,-278},{-122,-258}})));
  Modelica.Blocks.Sources.Constant const4(k=0)
    annotation (Placement(transformation(extent={{-166,-310},{-152,-296}})));
  Modelica.Blocks.Continuous.Integrator integrator(
    k=1/3600,
    initType=Modelica.Blocks.Types.Init.InitialState,
    y_start=0)
    annotation (Placement(transformation(extent={{-98,-278},{-78,-258}})));
  Modelica.Blocks.Continuous.Integrator integrator1(k=1/3600, initType=Modelica.Blocks.Types.Init.InitialState)
    annotation (Placement(transformation(extent={{-98,-354},{-78,-334}})));
  Modelica.Blocks.Math.Add KelvinHours_discomfort(k2=+1)
    annotation (Placement(transformation(extent={{-54,-316},{-34,-296}})));
  IDEAS.Fluid.HeatExchangers.HeaterCooler_u AuxiliaryHeater(
    m_flow_nominal=2,
    Q_flow_nominal=36000,
    redeclare package Medium = Medium,
    dp_nominal=0)
    annotation (Placement(transformation(extent={{144,40},{164,60}})));
  IDEAS.Fluid.Movers.FlowControlled_m_flow fanAuxiliaryHeater(
    redeclare package Medium = Medium,
    addPowerToMedium=false,
    tau=60,
    use_inputFilter=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    m_flow_nominal=2,
    allowFlowReversal=false,
    constantMassFlowRate=2,
    inputType=IDEAS.Fluid.Types.InputType.Continuous,
    T_start=308.15) annotation (Placement(transformation(
        extent={{-8,-8},{8,8}},
        rotation=0,
        origin={118,50})));
  Modelica.Blocks.Sources.Constant const5(k=0)
    annotation (Placement(transformation(extent={{164,82},{152,94}})));
  Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=heaPum.errLowPre)
    annotation (Placement(transformation(extent={{86,98},{106,118}})));
  Modelica.Blocks.Math.BooleanToReal booleanToReal2(realTrue=2.0) annotation (
      Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=-90,
        origin={116,74})));
  Modelica.Blocks.Logical.Switch switch8
    annotation (Placement(transformation(extent={{-5,5},{5,-5}},
        rotation=-90,
        origin={143,73})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort Eva_in(
    redeclare package Medium = Medium,
    tau=0,
    initType=Modelica.Blocks.Types.Init.SteadyState,
    m_flow_nominal=fanEmbeddedPipe.m_flow_nominal,
    T_start=308.15)
    annotation (Placement(transformation(extent={{-62,72},{-48,60}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort EvaOut(
    redeclare package Medium = Medium,
    tau=0,
    initType=Modelica.Blocks.Types.Init.SteadyState,
    m_flow_nominal=fanEmbeddedPipe.m_flow_nominal,
    T_start=308.15) annotation (Placement(transformation(
        extent={{7,6},{-7,-6}},
        rotation=90,
        origin={-25,10})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort Con_in(
    redeclare package Medium = Medium,
    tau=0,
    initType=Modelica.Blocks.Types.Init.SteadyState,
    m_flow_nominal=fanEmbeddedPipe.m_flow_nominal,
    T_start=308.15)
    annotation (Placement(transformation(extent={{12,34},{-2,22}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort Con_out(
    redeclare package Medium = Medium,
    tau=0,
    initType=Modelica.Blocks.Types.Init.SteadyState,
    m_flow_nominal=fanEmbeddedPipe.m_flow_nominal,
    T_start=308.15)
    annotation (Placement(transformation(extent={{-2,58},{12,46}})));
  Modelica.Blocks.Math.Gain gain2(k=fanEmbeddedPipe.m_flow_nominal)
                                       annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=0,
        origin={267,-221})));
  Modelica.Blocks.Math.Gain gain3(k=0.5)
                                       annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=0,
        origin={269,-253})));
equation
  connect(upscaleCase900_HVAC.heatPortEmb, embeddedPipe.heatPortEmb)
    annotation (Line(points={{-20,-164},{74,-164},{74,-108}},
                                                           color={191,0,0}));
  connect(fanBuffertank.port_b, BufferTank.ports[2]) annotation (Line(points={{36,52},
          {50.8286,52},{50.8286,62}},      color={0,127,255}));
  connect(fanBorefield.port_a, borFie.port_b) annotation (Line(points={{-78,-36},
          {-78,-42},{-64,-42}},      color={0,127,255}));
  connect(SwitchHeatCoolBorefield.port_b, fanBorefield.port_b)
    annotation (Line(points={{-78,-12},{-78,-18}},
                                                color={0,127,255}));
  connect(SwitchHeatCoolBorefield.port_a2, hex.port_a1)
    annotation (Line(points={{-70,-4},{-2,-4}},color={0,127,255}));
  connect(temperatureSensor.port, BufferTank.heatPort) annotation (Line(
        points={{20,129},{16,129},{16,84},{26,84},{26,89},{30,89}},
                                                  color={191,0,0}));
  connect(SwitchHeatCoolBorefield.switch, SwitchHeatCoolEmbeddedPipe.switch)
    annotation (Line(points={{-84.4,-4},{-116,-4},{-116,150},{172,150},{172,
          -4},{66.4,-4}},
                      color={255,0,255}));
  connect(Heating.y, not1.u)
    annotation (Line(points={{-275.4,120},{-249.2,120}}, color={255,0,255}));
  connect(not1.y, SwitchHeatCoolEmbeddedPipe.switch) annotation (Line(points={{-235.4,
          120},{-216,120},{-216,150},{172,150},{172,-4},{66.4,-4}},
                                                     color={255,0,255}));
  connect(gain.y, fanBuffertank.m_flow_in)
    annotation (Line(points={{-4,115.2},{-4,78},{28,78},{28,61.6}},
                                                 color={0,0,127}));
  connect(gain.u, booleanToReal.y) annotation (Line(points={{-4,133.6},{-4,
          142},{324,142},{324,27},{312.7,27}},
                                          color={0,0,127}));
  connect(booleanToReal1.u, or1.y)
    annotation (Line(points={{-173.4,49},{-179.3,49}}, color={255,0,255}));
  connect(booleanToReal1.y, gain1.u)
    annotation (Line(points={{-157.3,49},{-147.4,49}}, color={0,0,127}));
  connect(or1.u1, booleanToReal.u) annotation (Line(points={{-195.4,49},{-194,49},
          {-194,50},{-208,50},{-208,146},{318,146},{318,18},{290,18},{290,27},{296.6,
          27}},
        color={255,0,255}));
  connect(embeddedPipe.port_a, EmbeddedPipeIn.port_b)
    annotation (Line(points={{84,-98},{104,-98}},color={0,127,255}));
  connect(fanEmbeddedPipe.port_b, EmbeddedPipeIn.port_a) annotation (Line(
        points={{164,-90},{164,-98},{118,-98}},
                                             color={0,127,255}));
  connect(embeddedPipe.port_b, EmbeddedPipeOut.port_a)
    annotation (Line(points={{64,-98},{46,-98}}, color={0,127,255}));
  connect(threeWayValveTempSetoint.port_b, EmbeddedPipeOut.port_b)
    annotation (Line(points={{26,-62},{26,-98},{32,-98}},
        color={0,127,255}));
  connect(runningMeanTemperature6h.TRm, HeatingCurveIn.u) annotation (Line(
        points={{-239.46,-78},{-228,-78},{-228,-62},{-216,-62}}, color={0,0,
          127}));
  connect(BufferTank.ports[3], SwitchHeatCoolEmbeddedPipe.port_a1)
    annotation (Line(points={{53.9143,62},{60,62},{60,4}},color={0,127,255}));
  connect(bou1.ports[1], SwitchHeatCoolEmbeddedPipe.port_a1)
    annotation (Line(points={{78,13},{60,13},{60,4}},  color={0,127,255}));
  connect(EmbeddedPipeIn.T, conPIDINHeat.u_m) annotation (Line(points={{111,
          -104.6},{112,-104.6},{112,-128},{-92,-128},{-92,-71.6},{-164,-71.6}},
        color={0,0,127}));
  connect(threeWayValveTempSetoint.port_a2, jun.port_3) annotation (Line(
        points={{34,-54},{154,-54}},                  color={0,127,255}));
  connect(SwitchHeatCoolEmbeddedPipe.port_b, jun.port_1)
    annotation (Line(points={{60,-12},{60,-44},{164,-44}},
                                               color={0,127,255}));
  connect(fanEmbeddedPipe.port_a, jun.port_2) annotation (Line(points={{164,-74},
          {164,-64}},                       color={0,127,255}));
  connect(borFie.port_a, jun1.port_2)
    annotation (Line(points={{-44,-42},{-36,-42}}, color={0,127,255}));
  connect(hex.port_b1, jun1.port_1) annotation (Line(points={{-2,-24},{-8,-24},
          {-8,-42},{-16,-42}},color={0,127,255}));
  connect(threeWayValveTempSetoint.port_a1, hex.port_a2) annotation (Line(
        points={{26,-46},{18,-46},{18,-24},{10,-24}},
                                                    color={0,127,255}));
  connect(threeWayValveTempSetoint.port_a1, BufferTank.ports[4]) annotation (
      Line(points={{26,-46},{36,-46},{36,4},{56,4},{56,62},{57,62}},
                                                     color={0,127,255}));
  connect(HexOut.port_b, hex.port_b2)
    annotation (Line(points={{16,-4},{10,-4}}, color={0,127,255}));
  connect(HexOut.port_a, SwitchHeatCoolEmbeddedPipe.port_a2)
    annotation (Line(points={{30,-4},{52,-4}}, color={0,127,255}));
  connect(HeatingCurveOut.u, runningMeanTemperature6h1.TRm)
    annotation (Line(points={{174,-214},{154.54,-214}},color={0,0,127}));
  connect(Heating.y, switch1.u2) annotation (Line(points={{-275.4,120},{-268,
          120},{-268,-212},{116,-212},{116,-240},{304,-240}},
                                                            color={255,0,255}));
  connect(conPIDOutCool.u_m, conPIDOutHeat.u_m) annotation (Line(points={{230,
          -269.6},{230,-312},{82,-312},{82,-236},{226,-236},{226,-223.6}},
        color={0,0,127}));
  connect(CoolingCurveIn.u, HeatingCurveIn.u) annotation (Line(points={{-216,
          -120},{-228,-120},{-228,-62},{-216,-62}}, color={0,0,127}));
  connect(HeatingCurveIn.y, conPIDINHeat.u_s)
    annotation (Line(points={{-193,-62},{-173.6,-62}},color={0,0,127}));
  connect(CoolingCurveIn.y, conPIDINCool.u_s)
    annotation (Line(points={{-193,-120},{-175.6,-120}},color={0,0,127}));
  connect(conPIDINHeat.y, switch2.u1) annotation (Line(points={{-155.2,-62},{
          -138,-62},{-138,-88},{-126,-88}},
                                         color={0,0,127}));
  connect(conPIDINCool.y, switch2.u3) annotation (Line(points={{-157.2,-120},
          {-136,-120},{-136,-104},{-126,-104}},
                                            color={0,0,127}));
  connect(conPIDINCool.u_m, conPIDINHeat.u_m) annotation (Line(points={{-166,
          -129.6},{-166,-136},{-92,-136},{-92,-71.6},{-164,-71.6}},
        color={0,0,127}));
  connect(switch2.u2, switch1.u2) annotation (Line(points={{-126,-96},{-264,
          -96},{-264,-212},{116,-212},{116,-240},{304,-240}},
                                                            color={255,0,255}));
  connect(switch2.y, threeWayValveTempSetoint.ctrl) annotation (Line(points={{-103,
          -96},{-10,-96},{-10,-54},{17.36,-54}},    color={0,0,127}));
  connect(heatingCoolingSet1.SetpointCooling, Cooling.u) annotation (Line(
        points={{-311,95.6},{-306,95.6},{-306,96},{-300,96},{-300,77},{-293.4,
          77}}, color={0,0,127}));
  connect(Cooling.y, Neutral.u2) annotation (Line(points={{-277.3,77},{-252,
          77},{-252,93.2},{-249.2,93.2}}, color={255,0,255}));
  connect(heatingCoolingSet1.SetpointHeating, Heating.u) annotation (Line(
        points={{-311,98.8},{-300,98.8},{-300,120},{-289.2,120}}, color={0,0,
          127}));
  connect(Neutral.u1, Heating.y) annotation (Line(points={{-249.2,98},{-268,
          98},{-268,120},{-275.4,120}}, color={255,0,255}));
  connect(or1.u2, SwitchHeatCoolEmbeddedPipe.switch) annotation (Line(points={{-195.4,
          43.4},{-216,43.4},{-216,150},{172,150},{172,-4},{66.4,-4}},
        color={255,0,255}));
  connect(gain1.y, switch3.u1) annotation (Line(points={{-131.3,49},{-126,49},
          {-126,-19.6},{-121.6,-19.6}}, color={0,0,127}));
  connect(switch3.y, fanBorefield.m_flow_in) annotation (Line(points={{-103.2,
          -26},{-87.6,-26},{-87.6,-27}},color={0,0,127}));
  connect(Neutral.y, switch3.u2) annotation (Line(points={{-235.4,98},{-222,
          98},{-222,-26},{-121.6,-26}}, color={255,0,255}));
  connect(const.y, switch3.u3) annotation (Line(points={{-237.3,-33},{-163.65,
          -33},{-163.65,-32.4},{-121.6,-32.4}}, color={0,0,127}));
  connect(logicalSwitch.y, booleanToReal.u) annotation (Line(points={{282.8,2},{
          290,2},{290,27},{296.6,27}},  color={255,0,255}));
  connect(logicalSwitch.u2, switch3.u2) annotation (Line(points={{264.4,2},{
          180,2},{180,152},{-222,152},{-222,-26},{-121.6,-26}},
                                                              color={255,0,
          255}));
  connect(logicalSwitch.u3, booleanConstant.y) annotation (Line(points={{264.4,-4.4},
          {260,-4.4},{260,-4},{254,-4},{254,-17},{250.7,-17}},
                                                         color={255,0,255}));
  connect(switch4.y, fanEmbeddedPipe.m_flow_in)
    annotation (Line(points={{199.2,-66},{186,-66},{186,-82},{173.6,-82}},
                                                      color={0,0,127}));
  connect(switch4.u3, const1.y) annotation (Line(points={{217.6,-72.4},{217.6,
          -96},{237.2,-96}}, color={0,0,127}));
  connect(switch4.u2, switch3.u2) annotation (Line(points={{217.6,-66},{226,
          -66},{226,2},{180,2},{180,152},{-222,152},{-222,-26},{-121.6,-26}},
        color={255,0,255}));
  connect(add1.u1, conPIDINHeat.u_m) annotation (Line(points={{50,-270},{26,
          -270},{26,-128},{-92,-128},{-92,-71.6},{-164,-71.6}},color={0,0,127}));
  connect(add1.u2, EmbeddedPipeOut.T) annotation (Line(points={{50,-282},{39,
          -282},{39,-104.6}}, color={0,0,127}));
  connect(add1.y, conPIDOutHeat.u_m) annotation (Line(points={{73,-276},{82,
          -276},{82,-236},{226,-236},{226,-223.6}}, color={0,0,127}));
  connect(CoolingCurveOut.u, runningMeanTemperature6h2.TRm)
    annotation (Line(points={{178,-260},{160.54,-260}}, color={0,0,127}));
  connect(HeatingCurveOut.y, conPIDOutHeat.u_s)
    annotation (Line(points={{197,-214},{216.4,-214}}, color={0,0,127}));
  connect(conPIDOutCool.u_s, CoolingCurveOut.y)
    annotation (Line(points={{220.4,-260},{201,-260}}, color={0,0,127}));

  connect(logicalSwitch.u1, not1.u) annotation (Line(points={{264.4,8.4},{186,
          8.4},{186,156},{-268,156},{-268,120},{-249.2,120}},
        color={255,0,255}));
  connect(temperatureSensor.T, conPIDHeatPump.u_m) annotation (Line(points={{34,129},
          {194,129},{194,54},{252,54},{252,64.4}},       color={0,0,127}));
  connect(conPIDHeatPump.u_s, const2.y)
    annotation (Line(points={{242.4,74},{231,74}}, color={0,0,127}));
  connect(conPIDHeatPump.y, switch5.u1) annotation (Line(points={{260.8,74},{278,
          74},{278,74.4},{286.4,74.4}},     color={0,0,127}));
  connect(switch5.u2, not1.u) annotation (Line(points={{286.4,68},{276,68},{
          276,48},{186,48},{186,156},{-268,156},{-268,120},{-249.2,120}},
                        color={255,0,255}));
  connect(const3.y, switch5.u3) annotation (Line(points={{257,32},{286.4,32},{286.4,
          61.6}},        color={0,0,127}));
  connect(switch5.y, heaPum.y) annotation (Line(points={{304.8,68},{314,68},{314,
          154},{-44,154},{-44,22},{-17,22},{-17,30}},     color={0,0,127}));
  connect(upscaleCase900_HVAC.TSensor[1], Lowerbound_error.u1) annotation (Line(
        points={{-19.4,-176},{-10,-176},{-10,-238},{-352,-238},{-352,-268},{
          -298,-268}},
                  color={0,0,127}));
  connect(Upperbound_error.u2, Lowerbound_error.u1) annotation (Line(points={{-298,
          -336},{-352,-336},{-352,-268},{-298,-268}}, color={0,0,127}));
  connect(Lowerbound_error.y, lessThreshold.u)
    annotation (Line(points={{-275,-274},{-244,-274}}, color={0,0,127}));
  connect(lessThreshold1.u, Upperbound_error.y)
    annotation (Line(points={{-244,-330},{-275,-330}}, color={0,0,127}));
  connect(lessThreshold.y, switch7.u2) annotation (Line(points={{-221,-274},{
          -218,-274},{-218,-268},{-144,-268}}, color={255,0,255}));
  connect(switch6.u2, lessThreshold1.y) annotation (Line(points={{-142,-344},
          {-216,-344},{-216,-330},{-221,-330}},
                                          color={255,0,255}));
  connect(switch6.u1, switch7.u3) annotation (Line(points={{-142,-336},{-142,
          -276},{-144,-276}},
                        color={0,0,127}));
  connect(switch7.u1, lessThreshold.u) annotation (Line(points={{-144,-260},{
          -270,-260},{-270,-274},{-244,-274}}, color={0,0,127}));
  connect(switch6.u3, Upperbound_error.y) annotation (Line(points={{-142,-352},
          {-270,-352},{-270,-330},{-275,-330}},color={0,0,127}));
  connect(switch7.y, integrator.u)
    annotation (Line(points={{-121,-268},{-100,-268}},color={0,0,127}));
  connect(switch6.y, integrator1.u)
    annotation (Line(points={{-119,-344},{-100,-344}},color={0,0,127}));
  connect(const4.y, switch7.u3) annotation (Line(points={{-151.3,-303},{-142,
          -303},{-142,-276},{-144,-276}},
                                    color={0,0,127}));
  connect(integrator.y, KelvinHours_discomfort.u1) annotation (Line(points={{-77,
          -268},{-66,-268},{-66,-300},{-56,-300}}, color={0,0,127}));
  connect(integrator1.y, KelvinHours_discomfort.u2) annotation (Line(points={{-77,
          -344},{-66,-344},{-66,-312},{-56,-312}}, color={0,0,127}));
  connect(comfort.LowerComfort, Lowerbound_error.u2) annotation (Line(points={{-319.25,
          -293.2},{-308.625,-293.2},{-308.625,-280},{-298,-280}}, color={0,0,127}));
  connect(comfort.UpperComfort, Upperbound_error.u1) annotation (Line(points={{-319.25,
          -296.145},{-319.25,-296},{-308,-296},{-308,-324},{-298,-324}}, color={
          0,0,127}));
  connect(fanAuxiliaryHeater.port_b, AuxiliaryHeater.port_a)
    annotation (Line(points={{126,50},{144,50}}, color={0,127,255}));
  connect(fanAuxiliaryHeater.m_flow_in, booleanToReal2.y) annotation (Line(
        points={{118,59.6},{118,67.4},{116,67.4}}, color={0,0,127}));
  connect(booleanToReal2.u, booleanExpression.y) annotation (Line(points={{116,81.2},
          {116,108},{107,108}}, color={255,0,255}));
  connect(AuxiliaryHeater.u, switch8.y) annotation (Line(points={{142,56},{142,67.5},
          {143,67.5}}, color={0,0,127}));
  connect(const5.y, switch8.u3) annotation (Line(points={{151.4,88},{151.4,89},{
          147,89},{147,79}}, color={0,0,127}));
  connect(booleanExpression.y, switch8.u2) annotation (Line(points={{107,108},{143,
          108},{143,79}}, color={255,0,255}));
  connect(switch5.y, switch8.u1) annotation (Line(points={{304.8,68},{314,68},{314,
          154},{139,154},{139,79}}, color={0,0,127}));
  connect(AuxiliaryHeater.port_b, BufferTank.ports[5]) annotation (Line(
        points={{164,50},{166,50},{166,24},{60.0857,24},{60.0857,62}}, color=
          {0,127,255}));
  connect(fanAuxiliaryHeater.port_a, BufferTank.ports[6]) annotation (Line(
        points={{110,50},{70,50},{70,62},{63.1714,62}}, color={0,127,255}));
  connect(bou.ports[1], SwitchHeatCoolBorefield.port_a1)
    annotation (Line(points={{-90,72},{-78,72},{-78,4}}, color={0,127,255}));
  connect(Eva_in.port_a, SwitchHeatCoolBorefield.port_a1)
    annotation (Line(points={{-62,66},{-78,66},{-78,4}}, color={0,127,255}));
  connect(Eva_in.port_b, heaPum.port_a2) annotation (Line(points={{-48,66},{
          -26,66},{-26,52}}, color={0,127,255}));
  connect(EvaOut.port_a, heaPum.port_b2) annotation (Line(points={{-25,17},{
          -26,17},{-26,32}}, color={0,127,255}));
  connect(jun1.port_3, EvaOut.port_b)
    annotation (Line(points={{-26,-32},{-26,3},{-25,3}}, color={0,127,255}));
  connect(Con_in.port_a, BufferTank.ports[7]) annotation (Line(points={{12,28},
          {66.2571,28},{66.2571,62}}, color={0,127,255}));
  connect(Con_in.port_b, heaPum.port_a1)
    annotation (Line(points={{-2,28},{-14,28},{-14,32}}, color={0,127,255}));
  connect(heaPum.port_b1, Con_out.port_a)
    annotation (Line(points={{-14,52},{-2,52}}, color={0,127,255}));
  connect(fanBuffertank.port_a, Con_out.port_b)
    annotation (Line(points={{20,52},{12,52}}, color={0,127,255}));
  connect(conPIDOutHeat.y, gain2.u) annotation (Line(points={{234.8,-214},{
          246,-214},{246,-221},{258.6,-221}}, color={0,0,127}));
  connect(switch1.u1, gain2.y) annotation (Line(points={{304,-232},{290,-232},
          {290,-221},{274.7,-221}}, color={0,0,127}));
  connect(gain3.u, conPIDOutCool.y) annotation (Line(points={{260.6,-253},{
          250,-253},{250,-260},{238.8,-260}}, color={0,0,127}));
  connect(switch1.u3, gain3.y) annotation (Line(points={{304,-248},{290,-248},
          {290,-253},{276.7,-253}}, color={0,0,127}));
  connect(switch1.y, switch4.u1) annotation (Line(points={{327,-240},{332,
          -240},{332,-59.6},{217.6,-59.6}}, color={0,0,127}));
  annotation (Diagram(coordinateSystem(extent={{-360,-360},{340,160}})), Icon(
        coordinateSystem(extent={{-360,-360},{340,160}})));
end RBC_model_ASHRAE;
