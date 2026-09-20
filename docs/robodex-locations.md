# Robodex location evidence

Verified from public primary sources on **2026-09-19**. These are city/town-level
locations, not office entrances or an assertion that every job is based there.
The `locations.<region>.source_url` and `locations.<region>.verified` fields
preserve the source and verification date independently of the company's
careers-link check.

| Company | City/town | Primary source and evidence |
| --- | --- | --- |
| Boston Dynamics | Waltham, MA | [Careers](https://bostondynamics.com/careers/) lists “Waltham Office (POST)” and “Waltham, MA” alongside its open roles. |
| iRobot | Bedford, MA | [Terms and conditions](https://www.irobot.com/en_US/legal/terms-and-conditions.html) identifies “Corporate Headquarters, iRobot Corporation 8 Crosby Drive, Bedford, MA 01730”. |
| Formlabs | Somerville, MA | [Contact](https://formlabs.com/company/contact/) lists headquarters at “35 Medford St. Suite 201 Somerville, MA 02143, USA”. |
| Locus Robotics | Wilmington, MA | [Contact](https://locusrobotics.com/company/contact-us) lists “Global Headquarters 100 Fordham Rd Wilmington, MA 01887”. |
| Vecna Robotics | Waltham, MA | [Contact](https://www.vecnarobotics.com/contact-us/) lists “Our office 425 Waverley Oaks Road Waltham, MA 02452”. |
| Ambi Robotics | Berkeley, CA | [Contact](https://www.ambirobotics.com/contact/) lists “Ambi HQ 1610 Fifth Street Berkeley, CA 94710”. |
| Figure AI | San Jose, CA | [Careers](https://www.figure.ai/careers) says “our headquarters in San Jose, CA”. |

The original seed incorrectly used San Leandro for Ambi and Sunnyvale for
Figure. Locus Robotics also incorrectly linked to `locus.sh`, a different
company. Its official [website](https://locusrobotics.com/) and
[job openings](https://locusrobotics.com/company/careers/job-openings) were fetched
and verified against their page titles and content before correcting the links.

## Additional companies

For the expanded roster, see [host-derived company evidence](robodex-hosts.md)
and [additional company sources](robodex-company-sources.md). The
[multi-region and ROSCon audit](robodex-regional-sources.md) records newer
Generalist/MathWorks regional pins and the Eka/ROSCon additions. Each records
its own verification date, city source and coordinate provenance.

## Coordinate provenance

Coordinates are city/town representative points returned by the
[OpenStreetMap Nominatim search API](https://nominatim.openstreetmap.org/search)
on the same date, queried as `<city>, <state>, USA`. They are intentionally
marked `precision: "city"`. No office address was geocoded or inferred.

| Query | OSM relation | Latitude | Longitude |
| --- | --- | --- | --- |
| Waltham, Massachusetts | [1865772](https://www.openstreetmap.org/relation/1865772) | 42.3762385 | -71.2355644 |
| Bedford, Massachusetts | [1838335](https://www.openstreetmap.org/relation/1838335) | 42.4917301 | -71.2817947 |
| Somerville, Massachusetts | [1933746](https://www.openstreetmap.org/relation/1933746) | 42.3875968 | -71.0994968 |
| Wilmington, Massachusetts | [1842130](https://www.openstreetmap.org/relation/1842130) | 42.5464828 | -71.1736669 |
| Berkeley, California | [2833528](https://www.openstreetmap.org/relation/2833528) | 37.8708393 | -122.2728630 |
| San Jose, California | [112143](https://www.openstreetmap.org/relation/112143) | 37.3361663 | -121.8905910 |

Nominatim also returned New Bedford for the Bedford query; that result was
rejected. The selected result is explicitly Bedford, Middlesex County, MA.
Boston Dynamics and Vecna share the Waltham representative point; shared pins
must keep both companies discoverable, not invent separate office coordinates.

Coordinate data © [OpenStreetMap contributors](https://www.openstreetmap.org/copyright),
ODbL. Recheck primary sources when maintaining locations. Do not infer a relocation
from an isolated job posting in a different city.
