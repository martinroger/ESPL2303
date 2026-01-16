local pl2303_proto = Proto("pl2303", "Prolific PL2303")

-- Define Fields for our Tree
local f_desc = ProtoField.string("pl2303.desc", "Description")
local f_baud = ProtoField.uint32("pl2303.baud", "Baud Rate", base.DEC)

pl2303_proto.fields = { f_desc, f_baud }

-- Create 'Field' extractors for the standard USB headers
-- local usb_brequest = Field.new("usb.setup.bRequest")
-- local usb_wValue   = Field.new("usb.setup.wValue")
-- local usb_wIndex   = Field.new("usb.setup.wIndex")
-- local usb_wLength  = Field.new("usb.setup.wLength")
local usb_type     = Field.new("usb.bmRequestType")


function pl2303_proto.dissector(buffer, pinfo, tree)
    -- Extract the fields from the already-dissected USB layer
    -- local bReq_f = usb_brequest()
    -- local wVal_f = usb_wValue()
    -- local wIdx_f = usb_wIndex()
    -- local wLen_f = usb_wLength()
    local type_f = usb_type()

   
    -- We need at least the bRequest to know what we are looking at
    if not type_f then 
        print("No type_f")
        return 
        end
    -- print("bmReqType "..type_f.value)
    -- local bReq = bReq_f.value
    -- local wVal = wVal_f and wVal_f.value or 0
    -- local wIdx = wIdx_f and wIdx_f.value or 0
    -- local wLen = wLen_f and wLen_f.value or 0
    -- 
    -- 
    -- print(bReq,wVal,wIdx,wLen)
    
    local subtree = tree:add(pl2303_proto, buffer(), "PL2303 Vendor Details")
    pinfo.cols.protocol = "PL2303"

    local msg = "Unknown Transaction"
    
    if type_f.value == 0x40 then
        msg = "VENDOR WRITE Request"
    elseif type_f.value == 0xC0 then
        msg = "VENDOR READ Request"
    elseif type_f.value == 0x21 then
        print('0x21 : ',buffer(0,1))
        if buffer(0,1) == 0x20 then msg = "SET LINE Request"
        elseif buffer(0,1) == 0x22 then msg = "SET CONTROL Request"
        elseif buffer(0,1) == 0x23 then msg = "BREAK Request"
        end
    elseif type_f.value == 0xA1 then msg = "GET LINE Request"
    end

    -- 1. Identify the Request
    -- if bReq == 0x01 then
    --     if wVal == 0x8484 then msg = "Handshake Query (8484)"
    --     elseif wVal == 0x8383 then msg = "Handshake Query (8383)"
    --     elseif wVal == 0x0404 then msg = "Init Reg 0404, Val: " .. wIdx
    --     else msg = string.format("Vendor Access: Val 0x%04X, Idx 0x%04X", wVal, wIdx)
    --     end
    -- elseif bReq == 0x20 then msg = "SET_LINE_CODING"
    -- elseif bReq == 0x21 then msg = "GET_LINE_CODING"
    -- elseif bReq == 0x22 then
    --     msg = string.format("SET_CONTROL_LINE: DTR/RTS (Val: 0x%04X)", wVal)
    -- end

    -- 2. Handle the Payload (The 'buffer' passed to us)
    -- For SET_LINE_CODING, the 7 bytes are in the buffer
    -- if (bReq == 0x20 or bReq == 0x21) and buffer:len() >= 7 then
    --     local baud = buffer(0,4):le_uint()
    --     msg = msg .. string.format(" (%d baud)", baud)
    --     subtree:add(f_baud, buffer(0,4))
    -- end

    -- Update UI
    pinfo.cols.info:set("PL2303: " .. msg)
    subtree:add(f_desc, msg)
end

-- Registration
DissectorTable.get("usb.control"):add_for_decode_as(pl2303_proto)

local usb_product_table = DissectorTable.get("usb.product")
usb_product_table:add(0x067b2303, pl2303_proto)