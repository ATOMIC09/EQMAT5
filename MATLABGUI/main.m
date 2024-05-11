function main

    % Create GUI figure
    f = figure('Name','Real-time Audio Processing');
    
    % Add axes for plots (one for frequency response, one for time domain)
    axes_handle_freq = axes('Parent', f, 'Position', [0.3 0.3 0.6 0.5]);
    axes_handle_time = axes('Parent', f, 'Position', [0.1 0.1 0.6 0.2]);
    
    % Create audio recorder object
    try
        recorder = audiorecorder(44100, 16, 1); % Adjust as needed (Fs, bits per sample, channels)
    catch ME
        error_msg = ['Error creating audio recorder: ' ME.message];
        uiwait(msgbox(error_msg, 'Error'));
        return;
    end
    
    % Buffer to store captured audio data
    audio_buffer = zeros(recorder.TotalSamples, 1);
    
    % Default filter parameters
    filter_type = 'lowpass';
    Fc = 1000; % Cutoff frequency (adjustable later)
    
    % Button to start/stop recording
    start_button = uicontrol('Parent', f, 'Style', 'pushbutton',...
                             'String', 'Start Recording', 'Callback', {@start_recording, recorder});
    
    % Panel for filter selection
    filter_panel = uipanel('Parent', f, 'Title', 'Filter Selection', ...
                            'Position', [0.1 0.3 0.2 0.5]);
    
    % Dropdown menu for filter type
    filter_type_menu = uicontrol('Parent', filter_panel, 'Style', 'popupmenu',...
                                 'String', {'Low-pass', 'High-pass', 'Band-pass'}, ...
                                 'Value', 1, 'Callback', {@update_filter});
    
    % Edit box for cutoff frequency
    Fc_editbox = uicontrol('Parent', filter_panel, 'Style', 'edit',...
                           'String', num2str(Fc), 'Position', [10 50 50 20], ...
                           'Callback', {@update_filter});
    
    % Text label for cutoff frequency
    Fc_label = uicontrol('Parent', filter_panel, 'Style', 'text',...
                         'String', 'Cutoff Freq (Hz):', 'Position', [10 75 80 20]);
    
    % Audio level meter (dummy for now, replace with RMS calculation)
    audio_level_meter = uipanel('Parent', f, 'Title', 'Audio Level', ...
                                'Position', [0.8 0.3 0.1 0.5]); % Placeholder for level meter display
    
    % Timer to capture audio and update plot (adjust interval as needed)
    hTimer = timer('Period', 0.1, 'ExecutionMode', 'fixedRate', ...
                   'TimerFcn', @update_audio); % Assign the callback function here
    
    % Start the timer
    start(hTimer);
    
    % Function to handle filter selection and update
    function update_filter(~,~)
    
        % Get selected filter type
        filter_type_idx = get(filter_type_menu, 'Value');
        filter_type_list = get(filter_type_menu, 'String');
        filter_type = filter_type_list{filter_type_idx};

        
        % Get cutoff frequency from edit box
        try
            Fc = str2double(get(Fc_editbox, 'String'));
        catch ME
            warning('Invalid cutoff frequency entered. Using default value.');
            Fc = 1000;
            set(Fc_editbox, 'String', num2str(Fc));
        end
    end
    
    % Function to start recording (triggered by button or timer)
    function start_recording(~, ~, recorder)
        
        % Start/stop recording based on button state
        if strcmp(get(start_button, 'String'), 'Start Recording')
            start(recorder);
            set(start_button, 'String', 'Stop Recording');
        else
            stop(recorder);
            set(start_button, 'String', 'Start Recording');
        end
    end
    
    % Function executed periodically by the timer
    function update_audio(~,~)
        
        % Read captured audio data (continued from previous part)
        try
            audio_buffer = getaudiodata(recorder);
        catch ME
            stop(hTimer);
            error_msg = ['Error reading audio data: ' ME.message];
            uiwait(msgbox(error_msg, 'Error'));
            return;
        end
        
        % Process audio data
        [filtered_data, magnitude_response, f] = process_audio(audio_buffer);
    
        % Update frequency response plot
        cla(axes_handle_freq);
        plot(f, magnitude_response, 'Parent', axes_handle_freq);
        xlabel('Frequency (Hz)');
        ylabel('Magnitude');
        title('Real-time Frequency Response');
        % Limit y-axis for better visualization (adjust as needed)
        ylim([0 100]);
    
        % Update time domain plot (basic example, replace with desired visualization)
        cla(axes_handle_time);
        plot(audio_buffer, 'Parent', axes_handle_time);
        xlabel('Time Samples');
        ylabel('Amplitude');
        title('Time Domain');
        % Adjust axis limits as needed
    
    end
    
    function [filtered_data, magnitude_response, f] = process_audio(audio_data)
    
        Fs = 44100; % Sampling rate (assuming 44.1 kHz)
        
        % Apply filter based on selected type and cutoff frequency
        switch filter_type
            case 'lowpass'
                d = designfilt('lowpass', 'PassFrequency', Fc, 'SampleRate', Fs);
            case 'highpass'
                d = designfilt('highpass', 'PassFrequency', Fc, 'SampleRate', Fs);
            case 'bandpass'
                % Basic example for band-pass filter (replace with your desired design)
                Fbw = 1000; % Bandwidth (adjust as needed)
                d = designfilt('bandpass', 'HalfPowerFrequency1', Fc - Fbw/2, 'HalfPowerFrequency2', Fc + Fbw/2, 'SampleRate', Fs);
        end
    
        % Apply filter
        filtered_data = filtfilt(d, audio_data);
    
        % Calculate FFT
        fft_data = fft(filtered_data);
    
        % Get magnitude (absolute value) of complex FFT data
        magnitude_response = abs(fft_data);
    
        % Adjust for single-sided spectrum (optional)
        if size(fft_data, 1) > Fs/2
            magnitude_response = magnitude_response(1:Fs/2+1);
        end
    
        % Frequency vector for plotting
        f = Fs*(0:(size(magnitude_response, 1)-1))/Fs;
    end
end
