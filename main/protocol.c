
typedef unsigned char proto8_t;
typedef short proto16_t;
typedef struct proto_data_unit
{
    proto8_t* header;
    proto8_t* origin_tag;
    proto8_t* destination_tag;
    proto16_t length;
    proto8_t* payload;
    proto8_t header_size;
    proto8_t origin_tag_size;
    proto8_t destination_tag_size;
} proto_data_unit_t;

#define INCOMPLETE_BIT 0x80
#define VALUE_MASK 0x7F
#define PROTO8_MAX 0xFF
#define PROTO16_MAX 0xFFFF

#define PIR_PAYLOAD_SIZE(X) (((X -1) >> 3)+1)
#define LSENSOR_PAYLOAD_SIZE(X) (X*2)

#define UNK_ERR_HEADER_SIZE 1
#define OK_HEADER_SIZE 1
#define RPL_REQ_HEADER_SIZE 1
#define KEEPALIVE_HEADER_SIZE 1
#define PIR_HEADER_SIZE 1
#define LSENSOR_HEADER_SIZE 1

const proto8_t head_unknown_error[1] = {0};
const proto8_t head_ok[1] = {1};
const proto8_t head_reply_req[1] = {2};
const proto8_t head_keepalive[1] = {3};
const proto8_t head_send_pir[1] = {4};
const proto8_t head_send_lsensor[1] = {5};
const proto8_t head_send_ltrig[1] = {6};
const proto8_t head_idle[1] = {7};
const proto8_t head_wake[1] = {8};

proto8_t get_field_size(proto8_t field[])
{
    proto8_t i = 0;
    while (field[i] & INCOMPLETE_BIT)
    {
        i++;
    }
    return i+1;
}

char compare_header(const proto8_t header1[], proto8_t header2[])
{
    proto8_t i = 0;
    char equal = 1;
    proto8_t h1;
    proto8_t h2;
    do
    {
        if (header1[i] != header2[i])
        {
            equal = 0;
            break;
        }
        h1 = header1[i] & INCOMPLETE_BIT;
        h2 = header2[i] & INCOMPLETE_BIT;
        if (h1 ^ h2)
        {
            equal = 0;
            break;
        }
        i++;
    } while (h1 && h2);
    return equal;
}

proto_data_unit_t unmake_data_unit(unsigned char data_unit[])
{
    proto_data_unit_t pdu_elem;
    pdu_elem.header = &(data_unit[0]);
    pdu_elem.header_size = get_field_size(pdu_elem.header);
    pdu_elem.origin_tag = &(pdu_elem.header[pdu_elem.header_size]);
    pdu_elem.origin_tag_size = get_field_size(pdu_elem.origin_tag);
    pdu_elem.destination_tag = &(pdu_elem.origin_tag[pdu_elem.origin_tag_size]);
    pdu_elem.destination_tag_size = get_field_size(pdu_elem.destination_tag);
    pdu_elem.length = pdu_elem.destination_tag[pdu_elem.destination_tag_size] << 8;
    pdu_elem.length += pdu_elem.destination_tag[pdu_elem.destination_tag_size+1];
    if (pdu_elem.length)
        pdu_elem.payload = &(pdu_elem.destination_tag[pdu_elem.destination_tag_size+2]);
    else
        pdu_elem.payload = NULL;
    return pdu_elem;
}

static void assign_field(proto8_t destination[], const proto8_t field[], proto8_t* index)
{
    destination[*index] = field[0];
    (*index)++;
    for (proto8_t i = 0; field[i] & INCOMPLETE_BIT; i++)
    {
        destination[*index] = field[i];
        (*index)++;
    }
}

proto16_t make_data_unit(proto8_t data_unit[], const proto8_t header[], proto8_t origin_tag[], proto8_t destination_tag[], proto16_t payload_length, proto8_t payload[])
{
    proto8_t index = 0;
    assign_field(data_unit, header, &index);
    assign_field(data_unit, origin_tag, &index);
    assign_field(data_unit, destination_tag, &index);
    data_unit[index] = (payload_length >> 8) & PROTO8_MAX;
    data_unit[index+1] =payload_length & PROTO8_MAX;
    index += 2;
    for (proto8_t i = 0; i < payload_length; i++)
    {
        data_unit[index] = payload[i];
        index++;
    }
    return index;
}

proto16_t make_unknown_error_data_unit(proto8_t data_unit[], proto8_t origin_tag[], proto8_t destination_tag[])
{
    return make_data_unit(data_unit, head_unknown_error, origin_tag, destination_tag, 0, NULL);
}

proto16_t make_ok_data_unit(proto8_t data_unit[], proto8_t origin_tag[], proto8_t destination_tag[])
{
    return make_data_unit(data_unit, head_ok, origin_tag, destination_tag, 0, NULL);
}

proto16_t make_reply_req_data_unit(proto8_t data_unit[], proto8_t origin_tag[], proto8_t destination_tag[])
{
    return make_data_unit(data_unit, head_reply_req, origin_tag, destination_tag, 0, NULL);
}

proto16_t make_keepalive_data_unit(proto8_t data_unit[], proto8_t origin_tag[], proto8_t destination_tag[])
{
    return make_data_unit(data_unit, head_keepalive, origin_tag, destination_tag, 0, NULL);
}

proto16_t make_send_pir_data_unit(proto8_t data_unit[], proto8_t origin_tag[], proto8_t destination_tag[], proto8_t payload[], proto16_t size)
{
    return make_data_unit(data_unit, head_send_pir, origin_tag, destination_tag, size, payload);
}

proto16_t make_send_lsensor_data_unit(proto8_t data_unit[], proto8_t origin_tag[], proto8_t destination_tag[], proto8_t payload[], proto16_t size)
{
    return make_data_unit(data_unit, head_send_lsensor, origin_tag, destination_tag, size, payload);
}

proto16_t make_send_ltrig_data_unit(proto8_t data_unit[], proto8_t origin_tag[], proto8_t destination_tag[], proto8_t payload[], proto16_t size)
{
    return make_data_unit(data_unit, head_send_ltrig, origin_tag, destination_tag, size, payload);
}

proto16_t make_idle_data_unit(proto8_t data_unit[], proto8_t origin_tag[], proto8_t destination_tag[])
{
    return make_data_unit(data_unit, head_idle, origin_tag, destination_tag, 0, NULL);
}

proto16_t make_wake_data_unit(proto8_t data_unit[], proto8_t origin_tag[], proto8_t destination_tag[])
{
    return make_data_unit(data_unit, head_wake, origin_tag, destination_tag, 0, NULL);
}
