ObsInfo = rlNumericSpec([4 1]);
ObsInfo.Name = "Car State";
ObsInfo.Description ='x, y, e_err, ori';
% actDim = [1 1];
% ActInfo = rlNumericSpec(actDim,"LowerLimit",0.3,"UpperLimit",7.0);
ActInfo=rlFiniteSetSpec([0.5 1 1.5 2 2.5 3 3.5 4 4.5 5]);
ActInfo.Name="Car Action";

type myResetFunction.m;
type myStepFunction.m;

env=rlFunctionEnv(ObsInfo,ActInfo,"myStepFunction","myResetFunction");


obsInfo=getObservationInfo(env);
actInfo=getActionInfo(env);

rng(0)

dnn = [
    featureInputLayer(obsInfo.Dimension(1),'Normalization','none','Name','state')
    fullyConnectedLayer(24,'Name','CriticStateFC1')
    reluLayer('Name','CriticRelu1')
    fullyConnectedLayer(24, 'Name','CriticStateFC2')
    reluLayer('Name','CriticCommonRelu')
    fullyConnectedLayer(length(actInfo.Elements),'Name','output')];

figure
plot(layerGraph(dnn))

criticOpts = rlRepresentationOptions('LearnRate',0.001,'GradientThreshold',1);

critic = rlQValueRepresentation(dnn,obsInfo,actInfo,'Observation',{'state'},criticOpts);

agentOpts = rlDQNAgentOptions(...
    'UseDoubleDQN',false, ...    
    'TargetSmoothFactor',1, ...
    'TargetUpdateFrequency',4, ...   
    'ExperienceBufferLength',100000, ...
    'DiscountFactor',0.99, ...
    'MiniBatchSize',256);

agent = rlDQNAgent(critic,agentOpts);

trainOpts = rlTrainingOptions(...
    'MaxEpisodes',200, ...
    'MaxStepsPerEpisode',1000, ...
    'Verbose',false, ...
    'Plots','training-progress',...
    'StopTrainingCriteria','AverageReward',...
    'StopTrainingValue',1000); 

doTraining = true;
if doTraining    
    % Train the agent.
    trainingStats = train(agent,env,trainOpts);
else
    % Load the pretrained agent for the example.
    load('MATLABCartpoleDQNMulti.mat','agent');
end

simOptions = rlSimulationOptions('MaxSteps',500);
experience = sim(env,agent,simOptions);
totalReward = sum(experience.Reward)


