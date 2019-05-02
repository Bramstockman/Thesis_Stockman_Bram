within GIT_thesis_Bram_Stockman;
model RBC_model_EED_internal_heat_gains_ext10_good
  extends RBC_model_ASHRAE_internal_heat_gains_ext9(
   HeatingCurveIn(table=[-8.0 + 273,35 + 273; 15 + 273,22 + 273]),
    CoolingCurveIn(table=[20 + 273,20 + 273; 23 + 273,14 + 273]),
    fanEmbeddedPipe(m_flow_nominal=1.6),
    heatingCoolingSet1(                                    CoolSet=19 + 273, HeatSet=
          19 + 273),
    CoolingCurveOut(table=[15 + 273,0; 20 + 273,-5; 25 + 273,-7]),
    HeatingCurveOut(table=[-7 + 273,4.85; 5 + 273,3; 12 + 273,0]),
    occSch(occupancy=3600*{7,17,31,41,55,65,79,89,103,113,127,137,151,161}),
    gain4(k=fanEmbeddedPipe.m_flow_nominal/2),
    borFie(borFieDat=IDEAS.Fluid.Geothermal.Borefields.Data.Borefield.Example(
            filDat=IDEAS.Fluid.Geothermal.Borefields.Data.Filling.Bentonite(
              kFil=1.6,
              cFil=1200,
              dFil=1340,
              steadyState=true),
            soiDat=IDEAS.Fluid.Geothermal.Borefields.Data.Soil.SandStone(
              kSoi=1.9,
              cSoi=1364,
              dSoi=1760),
            conDat=
            IDEAS.Fluid.Geothermal.Borefields.Data.Configuration.Example(
              borCon=IDEAS.Fluid.Geothermal.Borefields.Types.BoreholeConfiguration.SingleUTube,
              use_Rb=true,
              Rb=0.0872,
              mBor_flow_nominal=1.9/4,
              mBorFie_flow_nominal=1.9,
              hBor=97,
              rBor=0.150,
              dBor=1,
              nBor=16,
              cooBor=[0,0; 0,13; 0,26; 0,39; 13,0; 13,13; 13,26; 13,39; 26,0;
              26,13; 26,26; 26,39; 39,0; 39,13; 39,26; 39,39],
              rTub=0.02,
              kTub=0.43,
              eTub=0.0036,
              xC=0.11))));
  Modelica.Blocks.Sources.RealExpression    realExpression(y=embeddedPipe.Q[1])
    annotation (Placement(transformation(extent={{372,112},{392,132}})));
  Modelica.Blocks.Logical.Switch CoolingDemand
    annotation (Placement(transformation(extent={{470,114},{486,130}})));
  Modelica.Blocks.Logical.LessThreshold lessThreshold2
    annotation (Placement(transformation(extent={{422,112},{442,132}})));
  Modelica.Blocks.Sources.Constant const9(k=0)
    annotation (Placement(transformation(extent={{424,74},{444,94}})));
  Modelica.Blocks.Logical.Switch HeatingDemand
    annotation (Placement(transformation(extent={{468,44},{484,60}})));
  Modelica.Blocks.Logical.LessThreshold lessThreshold3
    annotation (Placement(transformation(extent={{422,42},{442,62}})));
equation
  connect(realExpression.y, lessThreshold2.u)
    annotation (Line(points={{393,122},{420,122}}, color={0,0,127}));
  connect(lessThreshold2.y, CoolingDemand.u2)
    annotation (Line(points={{443,122},{468.4,122}}, color={255,0,255}));
  connect(CoolingDemand.u1, lessThreshold2.u) annotation (Line(points={{468.4,
          128.4},{462,128.4},{462,148},{406,148},{406,122},{420,122}}, color=
          {0,0,127}));
  connect(const9.y, CoolingDemand.u3) annotation (Line(points={{445,84},{456,
          84},{456,115.6},{468.4,115.6}}, color={0,0,127}));
  connect(HeatingDemand.u1, CoolingDemand.u3) annotation (Line(points={{466.4,
          58.4},{462,58.4},{462,58},{456,58},{456,115.6},{468.4,115.6}},
        color={0,0,127}));
  connect(lessThreshold3.y, HeatingDemand.u2)
    annotation (Line(points={{443,52},{466.4,52}}, color={255,0,255}));
  connect(lessThreshold3.u, lessThreshold2.u) annotation (Line(points={{420,
          52},{406,52},{406,122},{420,122}}, color={0,0,127}));
  connect(HeatingDemand.u3, lessThreshold2.u) annotation (Line(points={{466.4,
          45.6},{458,45.6},{458,26},{406,26},{406,122},{420,122}}, color={0,0,
          127}));
end RBC_model_EED_internal_heat_gains_ext10_good;
