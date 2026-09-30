from floodnet_hil.parse import parse_log

REC = "REC,66,5,1000,481173000,115166667,545400,8,0,0,0,0,0,-97,1,0,2,0,3700"


def test_tx_line():
    log = parse_log("100 TX,66,2,5,98,ok\n200 TX,66,2,6,198,timeout\n")
    assert [(t.key, t.completed, t.millis, t.t_ms) for t in log.tx] == [
        ((66, 2, 5), True, 98, 100),
        ((66, 2, 6), False, 198, 200),
    ]
    assert log.malformed == 0


def test_rec_line_key_and_rssi():
    log = parse_log(f"300 {REC}\n")
    assert len(log.rec) == 1
    assert log.rec[0].key == (66, 2, 5)
    assert log.rec[0].rssi == -97


def test_err_lines():
    log = parse_log("1 ERR,decode,1\n2 ERR,decode,2\n3 ERR,short,12,1\n")
    assert (log.decode_errors, log.short_frames, log.malformed) == (2, 1, 0)


def test_unknown_err_kind_is_malformed():
    assert parse_log("1 ERR,bogus,1\n").malformed == 1


def test_banners_are_ignored():
    log = parse_log("0 floodnet node: duty-cycled sampler\n1 radio: init failed\n")
    assert (len(log.tx), len(log.rec), log.malformed) == (0, 0, 0)


def test_bad_lines_are_malformed():
    text = "\n".join(
        [
            "x TX,66,2,5,98,ok",  # timestamp not a number
            "TX,66,2,5,98,ok",  # no timestamp
            "1 TX,66,2,5,98,maybe",  # unknown outcome
            "2 TX,66,2,5",  # too few fields
            "3 REC,66,5",  # too few fields
            "4 TX,66,two,5,98,ok",  # field not a number
        ]
    )
    assert parse_log(text + "\n").malformed == 6


def test_unterminated_final_line_is_malformed():
    log = parse_log("100 TX,66,2,5,98,ok\n200 TX,66,2,6,19")
    assert len(log.tx) == 1
    assert log.malformed == 1


def test_crlf_line_endings():
    log = parse_log("100 TX,66,2,5,98,ok\r\n")
    assert len(log.tx) == 1 and log.malformed == 0


def test_empty_log():
    log = parse_log("")
    assert (len(log.tx), len(log.rec), log.malformed) == (0, 0, 0)
