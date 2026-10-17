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
% Uso dell'inverseKinematics al posto di generalizedInverseKinematics
% ------------------------------
% Crea il solver di cinematica inversa standard
ik = inverseKinematics("RigidBodyTree", robot);

% Peso per la risoluzione: [x y z roll pitch yaw] (modifica se necessario)
weights = [0.25 0.25 0.25 1 1 1];

% Configurazione iniziale del robot (come struct o vettore, OK per inverseKinematics)
q0 = homeConfiguration(robot);

% Numero di pose da raggiungere.
numPoses = 4;

% Numero di giunti rappresentati nella configurazione.
numJoints = numel(q0);

% Prealloca la matrice delle configurazioni (riga per ogni posa)
Q = zeros(numPoses, numJoints);

% Memorizza le informazioni sul risultato del solver.
solInfoAll = cell(numPoses, 1);

% Funzione di supporto inline per convertire la soluzione in riga numerica
convertToRow = @(q) (isstruct(q) * 0); % placeholder per compatibilità; sovrascritto sotto

% Definisce convertToRow correttamente (cioè non usare function file separato)
if isstruct(q0)
    convertToRow = @(q) arrayfun(@(s)s.JointPosition, q);
else
    convertToRow = @(q) q;
end

% Calcola una configurazione per ciascuna posa con inverseKinematics
for i = 1:numPoses

    targetPose = Ttarget(:,:,i);

    % Risolvi la cinematica inversa
    % Restituisce qSol (struct array o vettore) e solInfo (info diagnostico)
    [qSol, solInfo] = ik(ee, targetPose, weights, q0);

    % Converti la soluzione in riga numerica, compatibile con Q
    try
        qRow = convertToRow(qSol);
    catch
        % Se qualcosa va storto con la conversione, prova ad estrarre manualmente
        if isstruct(qSol)
            qRow = zeros(1, numJoints);
            for k = 1:numel(qSol)
                if isfield(qSol(k), "JointPosition")
                    qRow(k) = qSol(k).JointPosition;
                else
                    qRow(k) = qSol(k);
                end
            end
        else
            qRow = qSol;
        end
    end

    % Controllo basilare sul risultato: solInfo può avere campi diversi a seconda della versione
    success = true;
    if isstruct(solInfo)
        if isfield(solInfo, "Status")
            success = strcmpi(solInfo.Status, "success");
        elseif isfield(solInfo, "ExitFlag")
            success = solInfo.ExitFlag > 0;
        end
        % Altre versioni di MATLAB potrebbero restituire campi diversi; non tutte forniscono "Status".
    end

    if ~success
        warning("Il solver inverseKinematics potrebbe non aver trovato una soluzione pienamente verificata per la posa %d. Controlla solInfoAll{%d}.", i, i);
        % Se preferisci terminare con errore, usa: error(...)
    end

    % Salva la configurazione numerica
    Q(i, :) = qRow;

    % Salva le informazioni del calcolo
    solInfoAll{i} = solInfo;

    % Usa la soluzione come punto iniziale successivo.
    % Per inverseKinematics l'initial guess può essere fornito come struct o vettore.
    if isstruct(qSol)
        q0 = qSol;
    else
        % Se qSol è vettore e robot.DataFormat="row", crea struct initial guess
        if isrow(qSol)
            % costruiamo un struct compatibile
            qStruct = homeConfiguration(robot);
            for k = 1:numel(qStruct)
                qStruct(k).JointPosition = qSol(k);
            end
            q0 = qStruct;
        else
            q0 = qSol;
        end
    end
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
