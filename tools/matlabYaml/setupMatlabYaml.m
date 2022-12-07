disp('Installing Matlab Yaml...')

disp('Adding snakeYaml to java path...');
snakeYamlPath = fullfile(pwd,'snakeyaml-1.10.jar');
if ~contains(javaclasspath('-static'),'snakeyaml-1.23.jar')
    jcp = fullfile(prefdir,'javaclasspath.txt');
    jcpID = fopen(jcp,'a');
    fprintf(jcpID,'\n%s',snakeYamlPath);
    fclose(jcpID);
    warning('Restart Matlab before using the Snake yaml library.');
else
    disp('SnakeYaml already present in java path.')
end
clearvars snakeYamlPath

disp('...done');