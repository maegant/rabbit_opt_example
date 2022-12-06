function [str] = yaml_write_file(filePath, domain, info, rowOnly)
%yaml_write_file Write 'obj' as YAML document to given 'filename', returns
%string representation.
%  rowOnly - if a numeric or cell array is a vector (size along one
%dimension is one) just put them all as row-vectors to keep things consistent.

% Original: http://code.google.com/p/yamlmatlab/
% Modified by: Eric Cousineau [ eacousineau@gmail ]
% Modified by (June 2022): Maegan Tucker [ mtucker@caltech.edu ] 

printinfo = true;
if nargin < 3
    info = [];
    printinfo = false;
end

if nargin < 4
	rowOnly = [];
end



domain_str = yaml_dump(domain, rowOnly);
info_str = yaml_dump(info, true);

% open file with write privledges
fid = fopen(filePath, 'w');

if printinfo
    % save gait information to yaml
    fprintf(fid, '%s\n','info:');
    fprintf(fid, '%s', info_str);
end

% save domain information
fprintf(fid, '%s\n','domain:');
fprintf(fid, '%s', domain_str);
fclose(fid);

end