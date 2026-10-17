clear;
clc;
close all;

% Importa il robot (sostituire con il percorso reale del file URDF)
robot = importrobot("PERCORSO_DEL_TUO_FILE.urdf");
robot.DataFormat = "row";

% Configurazione iniziale
config = homeConfiguration(robot);

showdetails(robot);

figure;
show(robot, config);
title("Robot nella configurazione iniziale");
robot.BodyNames

% Nome reale del frame dell'ee (pinza) da sostituire
ee = "NOME_REALE_DEL_FRAME_DELLA_PINZA";

% Coordinate illustrative: sostituirle con quelle reali.
% Unità: metri.
P = [
    0.30  0.10  0.25;  % 1. Avvicinamento all'oggetto
    0.30  0.10  0.15;  % 2. Presa dell'oggetto
    0.45  0.20  0.25;  % 3. Trasferimento sopra la destinazione
    0.45  0.20  0.15   % 4. Deposito
    ];

% Orientamenti illustrative, in radianti.
% Ogni riga contiene [yaw pitch roll] per la convenzione ZYX.
EUL = [
    0  pi  0;  % 1. Orientamento di avvicinamento
    0  pi  0;  % 2. Orientamento per la presa
    0  pi  0;  % 3. Orientamento durante il trasferimento
    0  pi  0   % 4. Orientamento per il deposito
    ];

% Crea le quattro matrici di trasformazione 4x4.
Ttarget = zeros(4, 4, 4);
for i = 1:4
    Tpos = trvec2tform(P(i,:));
    Trot = eul2tform(EUL(i,:), "ZYX");
    Ttarget(:,:,i) = Tpos * Trot;
end

% ------------------------------
% Uso di generalizedInverseKinematics con vincoli sui limiti dei giunti
% ------------------------------
% Crea il solver di cinematica inversa generalizzata
gik = generalizedInverseKinematics( ...
    "RigidBodyTree", robot, ...
    "ConstraintInputs", {"pose", "joint"});

% Crea il vincolo sulla posa della pinza.
poseConstraint = constraintPoseTarget(ee);

% Crea il vincolo sui limiti dei giunti.
jointConstraint = constraintJointBounds(robot);

% Configurazione iniziale del robot.
q0 = homeConfiguration(robot);

% Numero di pose da raggiungere.
numPoses = 4;

% Numero di giunti rappresentati nella configurazione.
numJoints = numel(q0);

% Prealloca la matrice delle configurazioni.
Q = zeros(numPoses, numJoints);

% Memorizza le informazioni sul risultato del solver.
solInfoAll = cell(numPoses, 1);

% Calcola una configurazione per ciascuna posa.
for i = 1:numPoses

    % Aggiorna la posa desiderata della pinza.
    poseConstraint.TargetTransform = Ttarget(:,:,i);

    % Risolvi la cinematica inversa con vincoli di giunto.
    [qSol, solInfo] = gik(q0, poseConstraint, jointConstraint);

    % Controlla se il solver ha trovato una soluzione entro le tolleranze definite.
    if isstruct(solInfo) && isfield(solInfo, "Status")
        if ~strcmp(solInfo.Status, "success")
            error("Soluzione non verificata per la posa %d. Stato: %s", i, solInfo.Status);
        end
    elseif isfield(solInfo, "ExitFlag")
        if solInfo.ExitFlag <= 0
            error("Soluzione non verificata per la posa %d. ExitFlag: %d", i, solInfo.ExitFlag);
        end
    end

    % Salva la configurazione nella riga corrispondente.
    Q(i,:) = qSol;

    % Salva le informazioni del calcolo.
    solInfoAll{i} = solInfo;

    % Usa la soluzione come punto iniziale successivo.
    q0 = qSol;

end

% Visualizza la matrice finale.
disp("Matrice delle configurazioni:");
disp(Q);

% Mostra la dimensione della matrice.
disp(size(Q));

% Salva le configurazioni e i dati associati in un file MATLAB.
save("configurazioni_PiPER.mat", ...
    "Q", "P", "EUL", "Ttarget", "solInfoAll");

% Esporta la sola matrice dei giunti in formato CSV.
writematrix(Q, "configurazioni_PiPER.csv");

% Verifica la cartella di lavoro corrente.
disp(pwd);

% Fine script.
