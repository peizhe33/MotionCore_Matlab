function signature=motioncore_plant_signature(plant)
% Compare plant structure, dialog parameters and source connections.
% Excludes outer Position, handles and model name; never changes the plant.
blocks=find_system(plant,'LookUnderMasks','all','FollowLinks','on','Type','Block');
blocks=sort(blocks(~strcmp(blocks,plant))); signature=cell(numel(blocks),1);
for k=1:numel(blocks)
    b=blocks{k}; item=struct();
    item.path=b(numel(plant)+2:end); item.type=get_param(b,'BlockType');
    pars=get_param(b,'DialogParameters'); keys=sort(fieldnames(pars));
    item.parameters=cell(numel(keys),2);
    for j=1:numel(keys),item.parameters(j,:)={keys{j},get_param(b,keys{j})};end
    ports=get_param(b,'PortHandles'); item.inputs=cell(numel(ports.Inport),1);
    for j=1:numel(ports.Inport)
        ln=get_param(ports.Inport(j),'Line'); src='unconnected';
        if ln~=-1
            ph=get_param(ln,'SrcPortHandle');
            if ph~=-1
                parent=get_param(ph,'Parent');
                src=sprintf('%s/%d',parent(numel(plant)+2:end),get_param(ph,'PortNumber'));
            end
        end
        item.inputs{j}=src;
    end
    signature{k}=item;
end
end
