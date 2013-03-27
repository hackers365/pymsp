function send_cmd() {
    var $this = $(this),
        url = $this.attr('url'),
        v = $this.attr('v');
    $.get(url, function(data) {
        $('#result').text(data);
    });
}
$(document)
.on('click', '._cmd', send_cmd);

function getinfo() {
    setTimeout(function() {
        $.get('/info', function(data) {
            $('#info').text(data);
            getinfo();
        });
    }, 1000);
};
getinfo();
