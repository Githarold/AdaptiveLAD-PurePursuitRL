% ObsInfo(State) 정의
ObsInfo = rlNumericSpec([4 1]);
ObsInfo.Name = "Car State";
ObsInfo.Description ='x, y, e_err, ori';

% 모델 선정 파라미터
Modelnumber=1;

if Modelnumber==1       % DQN
    % ActInfo 정의 및 ObsInfo, ActInfo, 초기화함수, 스탭함수를 인자로 환경생성
    ActInfo=rlFiniteSetSpec([0.5 1 1.5 2 2.5 3 3.5 4 4.5 5 5.5 6 6.5 7 7.5 8]);
    ActInfo.Name="Car Action";
    env=rlFunctionEnv(ObsInfo,ActInfo,"myStepFunction","myResetFunction");

    % 환경으로부터 obsInfo와 actInfo 재정의
    obsInfo=getObservationInfo(env);
    actInfo=getActionInfo(env);

    % Agent옵션 설정
    agentopts = rlDQNAgentOptions("SampleTime",0.1,"MiniBatchSize",32);

    % Critic신경망 설정
    learnopts = rlOptimizerOptions("LearnRate",1e-4,"GradientThreshold",10);
    agentopts.CriticOptimizerOptions = learnopts;

    % Agent 정의
    agent = rlDQNAgent(obsInfo, actInfo,agentopts);
elseif Modelnumber==2   % DDPG
    % ActInfo 정의 및 ObsInfo, ActInfo, 초기화함수, 스탭함수를 인자로 환경생성
    ActInfo=rlNumericSpec([1 1],"UpperLimit",8.0,"LowerLimit",0.1);
    ActInfo.Name="Car Action";
    env=rlFunctionEnv(ObsInfo,ActInfo,"myStepFunction","myResetFunction");

    % 환경으로부터 obsInfo와 actInfo 재정의
    obsInfo=getObservationInfo(env);
    actInfo=getActionInfo(env);

    % Agent옵션 설정
    agentopts = rlDDPGAgentOptions("SampleTime",0.1,"MiniBatchSize",32);
    
    % Actor신경망 설정
    learnopts = rlOptimizerOptions("LearnRate",1e-4,"GradientThreshold",10);
    agentopts.ActorOptimizerOptions = learnopts;
    
    % Critic신경망 설정
    learnopts = rlOptimizerOptions("LearnRate",1e-2,"GradientThreshold",10);
    agentopts.CriticOptimizerOptions = learnopts;

    % Agent 정의
    agent = rlDDPGAgent(obsInfo, actInfo,agentopts);
else                    % PPO
    % ActInfo 정의 및 ObsInfo, ActInfo, 초기화함수, 스탭함수를 인자로 환경생성
    ActInfo=rlFiniteSetSpec([0.5 1 1.5 2 2.5 3 3.5 4 4.5 5 5.5 6 6.5 7 7.5 8]);
    ActInfo.Name="Car Action";
    env=rlFunctionEnv(ObsInfo,ActInfo,"myStepFunction","myResetFunction");

    % 환경으로부터 obsInfo와 actInfo 재정의
    obsInfo=getObservationInfo(env);
    actInfo=getActionInfo(env);

    % Agent옵션 설정
    agentopts = rlPPOAgentOptions("SampleTime",0.1,"MiniBatchSize",32);
    
    % Actor신경망 설정
    learnopts = rlOptimizerOptions("LearnRate",1e-4,"GradientThreshold",10);
    agentopts.ActorOptimizerOptions = learnopts;
    
    % Critic신경망 설정
    learnopts = rlOptimizerOptions("LearnRate",1e-2,"GradientThreshold",10);
    agentopts.CriticOptimizerOptions = learnopts;

    % Agent 정의
    agent = rlPPOAgent(obsInfo, actInfo,agentopts);
end

% 훈련 환경 설정
trainOpts = rlTrainingOptions(...
    'MaxEpisodes',100, ...
    'MaxStepsPerEpisode',1000, ...
    'Verbose',false, ...
    'Plots','training-progress',...
    'StopTrainingCriteria','AverageReward',...
    'StopTrainingValue',100000000,...
    'UseParallel',1); 

% 학습여부 파라미터
doTraining = true;
if doTraining    
    % Agent 학습
    trainingStats = train(agent,env,trainOpts);
else
    % Agent 로드
    load('EX.mat','agent');
end

% 모델 시뮬레이션 및 리워드 계산
simOptions = rlSimulationOptions('MaxSteps',500);
experience = sim(env,agent,simOptions);
totalReward = sum(experience.Reward)