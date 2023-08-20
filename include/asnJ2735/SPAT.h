/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../J2735_201603.asn1"
 * 	`asn1c -S ../../skeletons -pdu=MessageFrame -fcompound-names -pdu=auto`
 */

#ifndef	_SPAT_H_
#define	_SPAT_H_


#include <asn_application.h>

/* Including external dependencies */
#include "MinuteOfTheYear.h"
#include "DescriptiveName.h"
#include "IntersectionStateList.h"
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct RegionalExtension;

/* SPAT */
typedef struct SPAT {
	MinuteOfTheYear_t	*timeStamp	/* OPTIONAL */;
	DescriptiveName_t	*name	/* OPTIONAL */;
	IntersectionStateList_t	 intersections;
	struct SPAT__regional {
		A_SEQUENCE_OF(struct RegionalExtension) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *regional;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SPAT_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SPAT;
extern asn_SEQUENCE_specifics_t asn_SPC_SPAT_specs_1;
extern asn_TYPE_member_t asn_MBR_SPAT_1[4];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "RegionalExtension.h"

#endif	/* _SPAT_H_ */
#include <asn_internal.h>
