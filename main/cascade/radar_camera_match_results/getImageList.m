function image_list = getImageList(Pic_path)
    fileList=dir(Pic_path);
    prefix = 'match';
    postfix = '.';
    image_list = {};
    for i=1:length(fileList) 
        if ~fileList(i).isdir
            image_file_name=fileList(i).name;
            prefix_index = strfind(fileList(i).name, prefix);
            prefix_len = size(prefix,2);
            postfix_index = strfind(fileList(i).name, postfix);
            postfix_len = size(postfix,2);
            image_id = image_file_name(prefix_index+prefix_len : postfix_index-postfix_len);
            image_id = str2num(image_id);
            image_list(image_id).name = image_file_name;
            image_list(image_id).path = strcat(Pic_path, image_file_name);
        end
    end

end
