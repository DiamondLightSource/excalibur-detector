
odin_data = {
  api_version: '0.1',
  ctrl_name: 'excalibur',
  current_page: '.home-view',
  adapter_list: [],
  adapter_objects: {},
  ctrl_connected: false,
  fp_connected: [false,false,false,false],
  acq_id: ''
  };


String.prototype.replaceAll = function(search, replacement) {
    var target = this;
    return target.replace(new RegExp(search, 'g'), replacement);
};

$.put = function(url, data, callback, type)
{
  if ( $.isFunction(data) ){
    type = type || callback,
    callback = data,
    data = {}
  }

  return $.ajax({
    url: url,
    type: 'PUT',
    success: callback,
    data: data,
    contentType: type
  });
}

function render(url)
{
  // This function decides what type of page to show
  // depending on the current url hash value.
  // Get the keyword from the url.
  var temp = "." + url.split('/')[1];
  // Hide whatever page is currently shown.
  $(".page").hide();

  // Show the new page
  $(temp).show();
  odin_data.current_page = temp;

}


$( document ).ready(function() 
{
  $("#fp-tabs").tabs();
  update_api_version();
  update_api_adapters();
  render(decodeURI(window.location.hash));

  setInterval(update_api_version, 5000);
  setInterval(update_detector_status, 1000);
  setInterval(update_fp_status, 1000);
  setInterval(update_fr_status, 1000);
//  setInterval(update_meta_status, 1000);
  setInterval(update_dacs, 5000);

  $('#set-hw-exposure').change(function(){
    update_exposure();
  });

  $('#set-hw-frames').change(function(){
    update_frames();
  });

  $('#set-hw-frames-per-trigger').change(function(){
    update_frames_per_trigger();
  });

  $('#set-hw-mode').change(function(){
    update_mode();
  });

  $('#set-hw-profile').change(function(){
    update_profile();
  });

  $('#detector-config-put-cmd').click(function(){
    detector_json_put_command();
  });

  $('#detector-config-get-cmd').click(function(){
    detector_json_get_command();
  });

  $('#fp-debug-level').change(function(){
    fp_debug_command();
    fr_debug_command();
  });

  $('#fr-reset-cmd').click(function(){
      fr_reset_statistics();
      fp_reset_statistics();
  });

  $(window).on('hashchange', function(){
    // On every hash change the render function is called with the new hash.
	// This is how the navigation of the app is executed.
	render(decodeURI(window.location.hash));
  });
});

function process_cmd_response(response)
{
}

function ctrl_command(command) {
    $.put('/api/' + odin_data.api_version + '/' + odin_data.ctrl_name + '/command/' + command, process_cmd_response);
}

function send_meta_command(command, params)
{
    $.ajax({
        url: '/api/' + odin_data.api_version + '/meta_listener/config/' + command,
        type: 'PUT',
        dataType: 'json',
        data: params,
        headers: {'Content-Type': 'application/json',
            'Accept': 'application/json'},
        success: process_cmd_response,
        async: false
    });
}

function send_fp_command(command, params)
{
    $.ajax({
        url: '/api/' + odin_data.api_version + '/fp/config/' + command,
        type: 'PUT',
        dataType: 'json',
        data: params,
        headers: {'Content-Type': 'application/json',
            'Accept': 'application/json'},
        success: process_cmd_response,
        async: false
    });
}

function send_fr_command(command, params)
{
    $.ajax({
        url: '/api/' + odin_data.api_version + '/fr/config/' + command,
        type: 'PUT',
        dataType: 'json',
        data: params,
        headers: {'Content-Type': 'application/json',
            'Accept': 'application/json'},
        success: process_cmd_response,
        async: false
    });
}

function fr_reset_statistics()
{
    $.ajax({
        url: '/api/' + odin_data.api_version + '/fr/command/reset_statistics',
        type: 'PUT',
        dataType: 'json',
        headers: {'Content-Type': 'application/json',
            'Accept': 'application/json'},
        success: process_cmd_response,
        async: false
    });
}

function fp_reset_statistics()
{
    $.ajax({
        url: '/api/' + odin_data.api_version + '/fp/command/reset_statistics',
        type: 'PUT',
        dataType: 'json',
        headers: {'Content-Type': 'application/json',
            'Accept': 'application/json'},
        success: process_cmd_response,
        async: false
    });
}

function update_api_version() {

    $.getJSON('/api', function(response) {
        $('#api-version').html(response.api);
        odin_data.api_version = response.api;
    });
}

function update_api_adapters() {
    $.getJSON('/api/' + odin_data.api_version + '/adapters/', function(response) {
        odin_data.adapter_list = response.adapters;
        adapter_list_html = response.adapters.join(", ");
        $('#api-adapters').html(adapter_list_html);
        //update_adapter_objects();
    });
}

function Encode(string) { 
    var i = string.length, 
        a = []; 

    while (i--) { 
        var iC = string[i].charCodeAt(); 
        if (iC < 65 || iC > 127 || (iC > 90 && iC < 97)) { 
            a[i] = '&#' + iC + ';'; 
        } else { 
            a[i] = string[i]; 
        } 
    } 
    return a.join(''); 
} 

function update_dacs() {
    $.getJSON('/api/' + odin_data.api_version + '/' + odin_data.ctrl_name + '/status/dacs', function(response) {
        //alert(JSON.stringify(response));
        // Check if the response contains DAC values
        if ($.isEmptyObject(response['value'])){
            // Clear out all values
            for (index = 1; index <= 6; index++){
                fem = 'fem'+index+'-';
                $('[id^='+fem+']').html('');
            }
        } else {
            // Loop over all possible fems
            for (index = 1; index <= 6; index++){
                fem = ''+index;
                if (!response['value'][fem]){
                    // This FEM is not present
                } else {
                    $('#fem'+fem+'-title').html('FEM '+fem);
                    dacs = Object.keys(response['value'][fem]);
                    // Loop over each DAC setting the values
                    for (const dac of dacs){
                        dac_id = '#fem'+fem+'-'+dac;
                        html = "<table><tr>";
                        html += "<td>[</td>";
                        html += "<td width=\"35px\" align=\"right\">"+response['value'][fem][dac][0]+",</td>";
                        for(tdindex=1; tdindex < 7; tdindex++){
                            html += "<td width=\"35px\" align=\"right\">"+response['value'][fem][dac][tdindex]+",</td>";
                        }
                        html += "<td width=\"35px\" align=\"right\">"+response['value'][fem][dac][7]+"</td><td>]</td>";
                        html += "</tr></table>";
                        $(dac_id).html(html);
//                        $(dac_id).html(Encode(JSON.stringify(response['value'][fem][dac])));
                    }
                }
            }
        }
    });
}

function update_detector_status() {
    $.getJSON('/api/' + odin_data.api_version + '/' + odin_data.ctrl_name + '/username', function(response) {
//        alert(response.endpoint);
        $('#detector-hw-username').html(response['value']);
    });
    $.getJSON('/api/' + odin_data.api_version + '/' + odin_data.ctrl_name + '/start_time', function(response) {
//        alert(response.endpoint);
        $('#detector-hw-start-time').html(response['value']);
    });
    $.getJSON('/api/' + odin_data.api_version + '/' + odin_data.ctrl_name + '/up_time', function(response) {
//        alert(response.endpoint);
        $('#detector-hw-up-time').html(response['value']);
    });
    $.getJSON('/api/' + odin_data.api_version + '/' + odin_data.ctrl_name + '/status/model', function(response) {
        $('#detector-hw-description').html(response['value']);
    });
}

function update_fp_status() {
    $.getJSON('/api/' + odin_data.api_version + '/fp/endpoint', function(response) {
//        alert(response.endpoint);
        $('#fp-endpoint').html(response.endpoint);
    });
    $.getJSON('/api/' + odin_data.api_version + '/fp/status/connected', function(response) {
        //alert(response['value']);
        $('#fp-connected-1').html(led_html(response['value'][0], 'green', 26));
        $('#fp-connected-2').html(led_html(response['value'][1], 'green', 26));
        $('#fp-connected-3').html(led_html(response['value'][2], 'green', 26));
        $('#fp-connected-4').html(led_html(response['value'][3], 'green', 26));
        if (response['value'][0] === false){
            odin_data.fp_connected[0] = false;
            $('#fp-hdf-writing').html('');
            $('#fp-hdf-file-path').html('');
            $('#fp-hdf-processes').html('');
            $('#fp-hdf-rank').html('');
            $('#set-fp-filename').val('');
            $('#set-fp-path').val('');
        } else {
            odin_data.fp_connected[0] = true;
        }
        if (response['value'][1]){
            if (response['value'][1] === false){
                odin_data.fp_connected[1] = false;
            } else {
                odin_data.fp_connected[1] = true;
            }
        }
        if (response['value'][2]){
            if (response['value'][2] === false){
                odin_data.fp_connected[2] = false;
            } else {
                odin_data.fp_connected[2] = true;
            }
        }
        if (response['value'][3]){
            if (response['value'][3] === false){
                odin_data.fp_connected[3] = false;
            } else {
                odin_data.fp_connected[3] = true;
            }
        }
    });
    $.getJSON('/api/' + odin_data.api_version + '/fp/status/hdf', function(response) {
//        alert(response['value']);
        
        if (odin_data.fp_connected[0] == true){
          update_fp_data(1, response['value'][0]);
        }
        if (odin_data.fp_connected[1] == true){
          update_fp_data(2, response['value'][1]);
        }
        if (odin_data.fp_connected[2] == true){
          update_fp_data(3, response['value'][2]);
        }
        if (odin_data.fp_connected[3] == true){
          update_fp_data(4, response['value'][3]);
        }
    });
}

function update_fp_data(index, data){
    $('#fp-processes-'+index).html('' + data.processes);
    $('#fp-rank-'+index).html('' + data.rank);
    $('#fp-writing-'+index).html(led_html(data.writing, 'green', 26));
    $('#fp-written-'+index).html('' + data.frames_written);
    if (index == 1){
        odin_data.acq_id = data['acquisition_id'];
    }
}

function update_fr_status() {
    $.getJSON('/api/' + odin_data.api_version + '/fr/status/buffers', function (response) {
        //alert(response['value'][0].empty);
        $('#fr-empty-buffers-1').html(response['value'][0].empty);
        if (response['value'][1]){
            $('#fr-empty-buffers-2').html(response['value'][1].empty);
        }
        if (response['value'][2]){
            $('#fr-empty-buffers-3').html(response['value'][2].empty);
        }
        if (response['value'][3]){
            $('#fr-empty-buffers-4').html(response['value'][3].empty);
        }
    });
    $.getJSON('/api/' + odin_data.api_version + '/fr/status/decoder/packets', function (response) {
        //alert(response['value']);
        $('#fr-packets-1').html(response['value'][0]);
        if (response['value'][1]){
            $('#fr-packets-2').html(response['value'][1]);
        }
        if (response['value'][2]){
            $('#fr-packets-3').html(response['value'][2]);
        }
        if (response['value'][3]){
            $('#fr-packets-4').html(response['value'][3]);
        }
    });
    $.getJSON('/api/' + odin_data.api_version + '/fr/status/connected', function(response) {
        //alert(response['value']);
        $('#fr-connected-1').html(led_html(response['value'][0], 'green', 26));
        if (response['value'][1]){
            $('#fr-connected-2').html(led_html(response['value'][1], 'green', 26));
        }
        if (response['value'][2]){
            $('#fr-connected-3').html(led_html(response['value'][2], 'green', 26));
        }
        if (response['value'][3]){
            $('#fr-connected-4').html(led_html(response['value'][3], 'green', 26));
        }
    });
}

function update_meta_status() {
    $.getJSON('/api/' + odin_data.api_version + '/meta_listener/status', function (response) {
        data = response['value'][0];
        //alert(JSON.stringify(data));
        //alert(JSON.stringify(odin_data.acq_id));
        $('#meta-init').html(led_html(data['connected'], 'green', 26));
        if ($.isEmptyObject(data['acquisitions'])){
          $('#meta-active').html(led_html(false, 'green', 26));
          $('#meta-writing').html(led_html(false, 'green', 26));
          $('#meta-out').html('');
          $('#meta-frames').html('0');
          $('#meta-writers').html('0');
        } else {
          $('#meta-active').html(led_html(true, 'green', 26));
          $('#meta-writing').html(led_html(data['acquisitions'][odin_data.acq_id]['writing'], 'green', 26));
          $('#meta-out').html(data['acquisitions'][odin_data.acq_id]['filename']);
          $('#meta-frames').html(data['acquisitions'][odin_data.acq_id]['written']);
          $('#meta-writers').html(data['acquisitions'][odin_data.acq_id]['num_processors']);
        }
    });
}

function led_html(value, colour, width)
{
  var html_text = "<img width=" + width + "px src=img/";
  if (value == 'true' || value === true){
    html_text +=  colour + "-led-on";
  } else {
    html_text += "led-off";
  }
  html_text += ".png></img>";
  return html_text;
}
