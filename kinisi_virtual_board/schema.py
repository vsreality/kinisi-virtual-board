"""Read wire definitions directly from the SDK/firmware commands.json contract."""
import hashlib
import json
import math
from pathlib import Path
import struct

TYPES = {"bool": "B", "uint8_t": "B", "int8_t": "b", "uint16_t": "H",
         "int16_t": "h", "uint32_t": "I", "int32_t": "i", "uint64_t": "Q",
         "int64_t": "q", "integer": "i", "double": "d"}


class Rejected(Exception):
    """A named firmware protocol error, encoded using the loaded schema."""


class Schema:
    def __init__(self, path):
        self.path = Path(path).resolve()
        raw = self.path.read_bytes()
        self.sha256 = hashlib.sha256(raw).hexdigest()
        self.document = json.loads(raw)
        self.version = tuple(map(int, self.document["version"].split(".")))
        if len(self.version) != 3 or self.version[0] != 2:
            raise ValueError("Only Kinisi protocol major 2 is supported")
        header = [(p["name"], p["type"]) for p in self.document["header"]["properties"]]
        if header != [("length", "uint8_t"), ("command", "uint8_t"), ("message_id", "uint16_t")]:
            raise ValueError(f"Unsupported wire header: {header}")
        self.commands = {}
        self.by_code = {}
        self.objects = {o["name"]: o["properties"] for o in self.document["objects"]}
        self.errors = {e["name"]: e["code"] for e in self.document["error_codes"]}
        self.topics = {t["id"]: t for t in self.document.get("topics", [])}
        self.topic_names = {t["name"]: t for t in self.topics.values()}
        for command in self.document["commands"]:
            code = int(command["code"], 0)
            name = command["command"]
            if code in self.by_code or name in self.commands or not 0 <= code <= 255:
                raise ValueError(f"Duplicate/invalid command: {name}")
            self.commands[name] = command
            self.by_code[code] = command
            if self.codec(command.get("properties", [])).size > 251:
                raise ValueError(f"Request too large: {name}")
            self.response_fields(command)  # Fail fast on unresolved response objects.

    @staticmethod
    def codec(fields):
        return struct.Struct("<" + "".join(TYPES[p["type"]] for p in fields))

    def code(self, name):
        return int(self.commands[name]["code"], 0)

    def response_fields(self, command):
        response = command.get("response")
        if not response:
            return []
        kind = response["type"]
        if kind in TYPES:
            return [response]
        return self.objects[response["name"] if kind == "object" else kind]

    def unpack(self, command, payload):
        fields = command.get("properties", [])
        codec = self.codec(fields)
        if len(payload) != codec.size:
            raise Rejected("INVALID_LENGTH")
        values = dict(zip((p["name"] for p in fields), codec.unpack(payload)))
        for field in fields:
            value = values[field["name"]]
            if isinstance(value, float) and not math.isfinite(value):
                raise Rejected("INVALID_ARGUMENT")
            if field["type"] == "bool" and value not in (0, 1):
                raise Rejected("INVALID_ARGUMENT")
            if "range" in field and not field["range"][0] <= value <= field["range"][1]:
                raise Rejected("INVALID_ARGUMENT")
            if field.get("enum") == "telemetry_topic" and value not in self.topics:
                raise Rejected("INVALID_ARGUMENT")
        return values

    def pack(self, fields, values):
        return self.codec(fields).pack(*(values[p["name"]] for p in fields))

    def request(self, name, message_id=1, **values):
        """Build a raw request for protocol tests; SDK integration uses SDK encoders."""
        command = self.commands[name]
        fields = command.get("properties", [])
        values = {p["name"]: values.get(p["name"], p.get("default", 0)) for p in fields}
        return self.frame(name, message_id, self.pack(fields, values))

    def frame(self, name, message_id, payload=b""):
        if len(payload) > 251:
            raise ValueError("Payload exceeds the firmware's 255-byte frame limit")
        return struct.pack("<BBH", len(payload) + 3, self.code(name), message_id) + payload
