"""
sensor_data.h 파일을 파싱하여 구조체 정의를 자동으로 추출한다.

규칙:
  - #define ID_<NAME> <number>  →  id 매핑 (NAME을 소문자로 변환하여 struct 이름과 매칭)
  - struct <name> { ... };      →  필드 파싱 (PacketHeader header 필드는 제외)
  - pragma pack(push,1) 가정   →  little-endian, 패딩 없음
"""

import re, struct, os

# C 타입 → Python struct 포맷 코드 (little-endian)
TYPE_MAP = {
    'uint8_t':  'B',
    'int8_t':   'b',
    'uint16_t': 'H',
    'int16_t':  'h',
    'uint32_t': 'I',
    'int32_t':  'i',
    'uint64_t': 'Q',
    'int64_t':  'q',
    'float':    'f',
    'double':   'd',
}


class StructDef:
    """파싱된 구조체 하나의 정의"""
    def __init__(self, name: str, pkt_id: int, fields: list, field_types: list,
                 fmt: str, size: int, header_field_count: int):
        self.name   = name         # 구조체 이름 (예: 'baro')
        self.pkt_id = pkt_id       # 패킷 ID (예: 1)
        self.fields = fields       # 필드 이름 리스트 (헤더 제외)
        self.field_types = field_types  # 필드 C타입 리스트 (헤더 제외)
        self.fmt    = fmt          # struct.unpack 포맷 문자열
        self.size   = size         # 전체 패킷 크기 (헤더 포함)
        self.header_field_count = header_field_count  # 헤더 필드 개수 (디코딩 시 스킵)

    def decode(self, raw: bytes) -> dict:
        """raw 바이트를 dict로 디코딩 (헤더 필드는 건너뜀)"""
        values = struct.unpack(self.fmt, raw[:self.size])
        return dict(zip(self.fields, values[self.header_field_count:]))

    def __repr__(self):
        return f"StructDef({self.name}, id={self.pkt_id}, size={self.size}, fields={self.fields})"


def _parse_struct_body(body: str):
    """구조체 본문에서 (fmt_codes, field_names) 를 추출"""
    fmt_codes = []
    fields = []
    for line in body.split('\n'):
        line = line.strip().rstrip(';').strip()
        if not line or line.startswith('//'):
            continue
        parts = line.split()
        if len(parts) < 2:
            continue
        ctype = parts[0]
        fname = parts[1].rstrip(';').strip()
        if '//' in fname:
            fname = fname.split('//')[0].strip()
        fmt_code = TYPE_MAP.get(ctype)
        if fmt_code is None:
            continue
        fmt_codes.append(fmt_code)
        fields.append(fname)
    return fmt_codes, fields


def parse_header(header_path: str) -> dict:
    """
    sensor_data.h를 파싱하여 {pkt_id: StructDef} 딕셔너리를 반환한다.
    PacketHeader 구조체도 자동으로 파싱하여 필드 수/포맷을 결정한다.
    """
    with open(header_path, 'r', encoding='utf-8') as f:
        text = f.read()

    # 1) ID define 추출: #define ID_BARO 1 → {'baro': 1}
    id_map = {}
    for m in re.finditer(r'#define\s+ID_(\w+)\s+(\d+)', text):
        name = m.group(1).lower()
        id_map[name] = int(m.group(2))

    # 2) 모든 구조체 추출
    struct_pattern = re.compile(
        r'struct\s+(\w+)\s*\{([^}]+)\}', re.DOTALL
    )
    raw_structs = {}
    for m in struct_pattern.finditer(text):
        raw_structs[m.group(1)] = m.group(2)

    # 3) PacketHeader 파싱 — 필드 수와 포맷을 자동으로 결정
    header_fmt_codes = []
    header_field_count = 0
    if 'PacketHeader' in raw_structs:
        header_fmt_codes, _ = _parse_struct_body(raw_structs['PacketHeader'])
        header_field_count = len(header_fmt_codes)

    # 4) 센서 구조체 파싱
    structs = {}
    for sname, body in raw_structs.items():
        if sname == 'PacketHeader':
            continue

        pkt_id = id_map.get(sname.lower())
        if pkt_id is None:
            continue

        # 필드 파싱
        fields = []
        field_types = []
        fmt_codes = ['<']  # little-endian

        for line in body.split('\n'):
            line = line.strip().rstrip(';').strip()
            if not line or line.startswith('//'):
                continue

            # PacketHeader 필드 → 자동 파싱된 헤더 포맷 사용
            if 'PacketHeader' in line:
                fmt_codes.extend(header_fmt_codes)
                continue

            parts = line.split()
            if len(parts) < 2:
                continue
            ctype = parts[0]
            fname = parts[1].rstrip(';').strip()
            if '//' in fname:
                fname = fname.split('//')[0].strip()

            fmt_code = TYPE_MAP.get(ctype)
            if fmt_code is None:
                continue

            fmt_codes.append(fmt_code)
            fields.append(fname)
            field_types.append(ctype)

        fmt_str = ''.join(fmt_codes)
        size = struct.calcsize(fmt_str)

        structs[pkt_id] = StructDef(
            name=sname,
            pkt_id=pkt_id,
            fields=fields,
            field_types=field_types,
            fmt=fmt_str,
            size=size,
            header_field_count=header_field_count,
        )

    return structs


def get_all_field_names(structs: dict) -> list:
    """모든 구조체의 필드 이름을 합집합으로 반환 (순서 유지)"""
    seen = set()
    result = []
    for sd in structs.values():
        for f in sd.fields:
            if f not in seen:
                seen.add(f)
                result.append(f)
    return result


if __name__ == '__main__':
    # 테스트
    here = os.path.dirname(os.path.abspath(__file__))
    defs = parse_header(os.path.join(here, 'sensor_data.h'))
    for pid, sd in sorted(defs.items()):
        print(sd)
        for name, ctype in zip(sd.fields, sd.field_types):
            print(f'  {name:<20s} {ctype}')
