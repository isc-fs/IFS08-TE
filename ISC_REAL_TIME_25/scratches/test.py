import iso8601
import pytz
_date_obj=iso8601.parse_date("2018-09-07T04:57:58.050-07:00")
print(type(_date_obj))
_date_utc=_date_obj.astimezone(pytz.utc)
_date_utc_zformat=_date_utc.strftime('%Y-%m-%dT%H:%M:%SZ')
print(type(_date_utc_zformat))