% pick_and_place_4_config.m
% Versione estesa: usa generalizedInverseKinematics con vincoli sui limiti dei giunti,
% include debug, controllo automatico delle pose e possibilità di impostare
% algoritmo e parametri del solver.

clear;
clc;
close all;

%% NOTE PRE-ESECUZIONE (consigliate da eseguire con Python/ROS/SDK)
% Esegui questi controlli prima di muovere fisicamente il robot:
% 1) ping all'indirizzo del controller del robot
% 2) verifica che ROS/daemon sia in esecuzione (es. /joint_states attivo)
% 3) controlla che non ci siano fault (topic /diagnostics o API dedicata)
% 4) esegui un 'soft move' di test molto piccolo e controlla risposta in joint_states
% (Per snippet Python/ROS2 vedi le istruzioni fornite separatamente)

%% Parametri di controllo e debug
verbose = true;
translation_tol = 5e-3;    % [m] tolleranza di posizione
rotation_tol = deg2rad(2); % [rad] tolleranza orientamento
maxRetries = 5;            % quante volte ritentare con jitter
jitterScale = 0.02;        % scala del jitter in radianti (per retry)
logCSVname = "diagnostic_pose_log.csv";

%% Importa robot
robot = importrobot("PERCORSO_DEL_TUO_FILE.urdf");
robot.DataFormat = "row";

% Configurazione iniziale (home)
q0 = homeConfiguration(robot);

showdetails(robot);
figure;
show(robot, q0);
title("Robot nella configurazione iniziale");

% Nome reale dell'end-effector (sostituisci con il nome reale)
ee = "NOME_REALE_DEL_FRAME_DELLA_PINZA";

% Punti target (m) e orientazioni (eul ZYX) devo mettere le posizion i e l 'orientazione desiderate x ogni configurazione della pinza, quindi per tutte e 4 le pose intermedie
P = [
    0.30  0.10  0.25;
    0.30  0.10  0.15;
    0.45  0.20  0.25;
    0.45  0.20  0.15
    ];
EUL = [
    0  pi  0;
    0  pi  0;
    0  pi  0;
    0  pi  0
    ];

numPoses = size(P,1);

% Crea le trasformazioni target
Ttarget = zeros(4,4,numPoses);
for i = 1:numPoses
    Tpos = trvec2tform(P(i,:));
    Trot = eul2tform(EUL(i,:), "ZYX");
    Ttarget(:,:,i) = Tpos * Trot;
end

% ------------------------------
% Setup generalizedInverseKinematics e vincoli
% ------------------------------
% Crea il solver di cinematica inversa generalizzata
gik = generalizedInverseKinematics( ...
    "RigidBodyTree", robot, ...
    "ConstraintInputs", {"pose", "joint"});

% ----- Impostazione dell'algoritmo e dei parametri del solver (opzionale) -----
% Nota: non tutte le versioni della toolbox espongono tutte le opzioni; racchiudi in try/catch per compatibilità.
try
    % Scegli l'algoritmo solver: esempio "BFGSGradientProjection"
    gik.SolverAlgorithm = "BFGSGradientProjection";

    % Modifica i parametri del solver
    params = gik.SolverParameters;
    params.MaxIterations      = 1500;    % massimo numero di iterazioni
    params.SolutionTolerance  = 1e-6;    % tolleranza sulla soluzione
    params.GradientTolerance  = 1e-8;    % tolleranza sul gradiente
    params.MaxTime            = 10;      % tempo massimo per chiamata (s)
    gik.SolverParameters = params;
    if verbose
        disp("SolverAlgorithm impostato a: " + gik.SolverAlgorithm);
        disp(params);
    end
catch ME
    warning("Impostazione algoritmo/parametri del solver non supportata: %s", ME.message);
end
% ---------------------------------------------------------------------------

% Crea il vincolo sulla posa della pinza.
poseConstraint = constraintPoseTarget(ee);

% Crea il vincolo sui limiti dei giunti.
jointConstraint = constraintJointBounds(robot);

% (Opzionale) Imposta una initial guess esplicita come struct
initialGuess = homeConfiguration(robot);

% Esempio di chiamata singola al solver con visualizzazione del risultato (test)
try
    % usa la prima target come test (non altera il ciclo principale)
    poseConstraint.TargetTransform = Ttarget(:,:,1);
    [qSolTest, solInfoTest] = gik(initialGuess, poseConstraint, jointConstraint);

    disp("Configurazione dei giunti trovata (test):");
    disp(qSolTest);

    if isstruct(solInfoTest) && isfield(solInfoTest, "Status")
        disp("Stato del solver (test):");
        disp(solInfoTest.Status);
    elseif isfield(solInfoTest, "ExitFlag")
        disp("ExitFlag (test):");
        disp(solInfoTest.ExitFlag);
    end

    if isstruct(solInfoTest) && isfield(solInfoTest, "PoseErrorNorm")
        disp("Errore della posa (norm) (test):");
        disp(solInfoTest.PoseErrorNorm);
    end

    % Visualizza la configurazione trovata
    figure;
    show(robot, qSolTest);
    title("Soluzione Generalized Inverse Kinematics (test)");
catch ME
    warning("Chiamata di test al solver fallita: %s", ME.message);
end

% Estrai limiti dei giunti (ordine non-fixed bodies)
[lowerLimits, upperLimits] = getJointLimits(robot);
numJoints = numel(lowerLimits);

% Preallocazioni
Q = zeros(numPoses, numJoints);
solInfoAll = cell(numPoses,1);
diagRows = []; % per CSV: poseIndex, success, t_err, r_err_deg, attempts

% Ciclo sulle pose con retry e verifica
q0 = initialGuess; % start guess
for i = 1:numPoses
    target = Ttarget(:,:,i);
    poseConstraint.TargetTransform = target;
    successFlag = false;
    attempts = 0;
    lastSol = zeros(1,numJoints);
    lastSolInfo = [];

    while ~successFlag && attempts < maxRetries
        attempts = attempts + 1;
        if verbose
            fprintf("Pose %d: attempt %d\n", i, attempts);
        end

        % risolvi
        [qSol, solInfo] = gik(q0, poseConstraint, jointConstraint);

        % converti qSol in vettore riga (se è struct)
        if isstruct(qSol)
            qVec = arrayfun(@(s)s.JointPosition, qSol);
        else
            qVec = qSol;
        end

        % controlla limiti (doppio controllo)
        outOfBounds = any(qVec < lowerLimits - 1e-12) || any(qVec > upperLimits + 1e-12);

        % calcola T attuale e errori
        try
            Tact = getTransform(robot, qVec, ee);
        catch ME
            Tact = eye(4);
            warning("Impossibile calcolare getTransform: %s", ME.message);
        end
        [t_err, r_err] = poseError(Tact, target);

        % decide successo: solInfo + soglie errori + limiti
        okSolver = true;
        if isstruct(solInfo) && isfield(solInfo,"Status")
            okSolver = strcmp(solInfo.Status,"success");
        elseif isfield(solInfo,"ExitFlag")
            okSolver = solInfo.ExitFlag > 0;
        end

        if okSolver && ~outOfBounds && t_err <= translation_tol && r_err <= rotation_tol
            % accettata
            successFlag = true;
            lastSol = qVec;
            lastSolInfo = solInfo;
            if verbose
                fprintf("Pose %d OK (t_err=%.4fm r_err=%.3fdeg)\n", i, t_err, rad2deg(r_err));
            end
        else
            % log e ritenta: jitter sulla q0
            lastSol = qVec;
            lastSolInfo = solInfo;
            if verbose
                fprintf("Pose %d not OK: solver_ok=%d outOfBounds=%d t_err=%.4fm r_err=%.3fdeg\n", ...
                    i, okSolver, outOfBounds, t_err, rad2deg(r_err));
            end
            % prepara nuova initial guess: jitter intorno a qVec
            jitter = jitterScale * randn(1, numJoints);
            q0_vec = qVec; % partire dall'ultima soluzione
            % clip jittered guess ai limiti
            q0_vec = min(max(q0_vec + jitter, lowerLimits), upperLimits);
            % converti a struct
            q0 = vecToConfigStruct(q0_vec, robot);
        end
    end % while

    if ~successFlag
        warning("Pose %d: non raggiunta entro %d tentativi. Ultimi errori t=%.4fm r=%.3fdeg", ...
            i, maxRetries, t_err, rad2deg(r_err));
    end

    % salva i risultati per la posa i
    Q(i,:) = lastSol;
    solInfoAll{i} = lastSolInfo;

    % aggiorna q0 per la posa successiva (usa la soluzione trovata)
    q0 = vecToConfigStruct(lastSol, robot);

    % aggiungi riga diagnostica
    diagRows = [diagRows; i, successFlag, t_err, rad2deg(r_err), attempts];
end

% stampa e salva
disp("Matrice delle configurazioni:");
disp(Q);
disp(size(Q));

save("configurazioni_PiPER_debug.mat", "Q", "P", "EUL", "Ttarget", "solInfoAll", "diagRows");

% Salva CSV delle configurazioni con header (nomi giunti)
% Estrae i nomi dei giunti (ordine come in getJointLimits)
jointNames = {};
for b = 1:numel(robot.Bodies)
    j = robot.Bodies{b}.Joint;
    if ~strcmp(j.Type, "fixed")
        jointNames{end+1} = j.Name; %#ok<SAGROW>
    end
end
cfgCsv = "configurazioni_PiPER_with_header.csv";
fid = fopen(cfgCsv,'w');
fprintf(fid, "%s\n", strjoin(jointNames, ','));
fclose(fid);
% Scrive la matrice Q sotto l'header
try
    writematrix(Q, cfgCsv, 'Delimiter', ',', 'WriteMode', 'append');
catch
    % Per versioni MATLAB più vecchie, usare dlmwrite
    dlmwrite(cfgCsv, Q, '-append');
end

% salva CSV diagnostico esteso
% colonne: poseIndex, successFlag, translationError_m, rotationError_deg, attempts, PoseErrorNorm, ExitFlag
diagExtended = NaN(size(diagRows,1), 7);
for ii = 1:size(diagRows,1)
    poseIdx = diagRows(ii,1);
    succ = diagRows(ii,2);
    t_err = diagRows(ii,3);
    r_err_deg = diagRows(ii,4);
    attempts = diagRows(ii,5);
    s = solInfoAll{ii};
    if isstruct(s) && isfield(s, 'PoseErrorNorm')
        pnorm = s.PoseErrorNorm;
    else
        pnorm = NaN;
    end
    if isstruct(s) && isfield(s, 'ExitFlag')
        exitf = s.ExitFlag;
    else
        exitf = NaN;
    end
    diagExtended(ii,:) = [poseIdx, succ, t_err, r_err_deg, attempts, pnorm, exitf];
end

diagCsvExt = "diagnostic_pose_log_ext.csv";
hdrExt = "poseIndex,successFlag,translationError_m,rotationError_deg,attempts,PoseErrorNorm,ExitFlag";
fid = fopen(diagCsvExt,'w');
fprintf(fid, "%s\n", hdrExt);
fclose(fid);
try
    writematrix(diagExtended, diagCsvExt, 'Delimiter', ',', 'WriteMode', 'append');
catch
    dlmwrite(diagCsvExt, diagExtended, '-append');
end

% Prova a salvare solInfoAll in formato JSON per debug (opzionale)
try
    jsonText = jsonencode(solInfoAll);
    fid = fopen('solInfoAll.json','w');
    fprintf(fid, '%s', jsonText);
    fclose(fid);
catch ME
    warning('Impossibile salvare solInfoAll come JSON: %s', ME.message);
end

% mantiene compatibilità con la variabile di log precedente
fid = fopen(logCSVname,'w');
fprintf(fid, "%s\n", "poseIndex,successFlag,translationError_m,rotationError_deg,attempts");
fclose(fid);
try
    dlmwrite(logCSVname, diagRows, '-append');
catch
    % ignore
end

disp("Log salvati:");
disp(pwd);
disp(cfgCsv);
disp(diagCsvExt);

%% Funzioni di supporto

function [lower, upper] = getJointLimits(robot)
    lower = [];
    upper = [];
    for b = 1:numel(robot.Bodies)
        j = robot.Bodies{b}.Joint;
        if ~strcmp(j.Type, "fixed")
            lim = j.PositionLimits; % [low high]
            lower(end+1) = lim(1);
            upper(end+1) = lim(2);
        end
    end
    % ensure row vectors
    lower = reshape(lower,1,[]);
    upper = reshape(upper,1,[]);
end

function cfg = vecToConfigStruct(qVec, robot)
    % converte un vettore qVec in struct array compatibile con homeConfiguration
    base = homeConfiguration(robot);
    for k = 1:numel(base)
        base(k).JointPosition = qVec(k);
    end
    cfg = base;
end

function [t_err, r_err] = poseError(Tact, Ttarget)
    % ritorna translation error (m) e rotation error (rad)
    p_act = Tact(1:3,4);
    p_tgt = Ttarget(1:3,4);
    t_err = norm(p_act - p_tgt);

    Ract = Tact(1:3,1:3);
    Rtgt = Ttarget(1:3,1:3);
    Rdiff = Ract * Rtgt';
    % numerically stable trace->angle
    c = (trace(Rdiff) - 1) / 2;
    c = max(-1, min(1, c));
    r_err = acos(c);
end
